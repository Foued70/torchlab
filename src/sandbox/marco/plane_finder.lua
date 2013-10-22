fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual

pf = Class()

function pf.compute_bbx(idx,win_h,win_w,img_h,img_w)
   local hwin_h  = win_h / 2
   local hwin_w  = win_w / 2
   local near_h  = math.max(1,idx[1]-hwin_h)
   local far_h   = math.min(img_h-hwin_h,near_h+win_h)
   local near_w  = math.max(1,idx[2]-hwin_w)
   local far_w   = math.min(img_w-hwin_w,near_w+win_w)
   return {near_h, far_h, near_w, far_w }
end

function pf.draw_window(rgb,idx,win_w,win_h)
   local dims    = rgb:nDimension()
   local img_w   = rgb:size(dims)
   local img_h   = rgb:size(dims-1)
   local bbx     = compute_bbx(idx,win_h, win_w, img_h, img_w)
   local near_h  = bbx[1]
   local far_h   = bbx[2]
   local near_w  = bbx[3]
   local far_w   = bbx[4]
   rgb[{{},{near_h,far_h},{near_w,near_w+1}}] = 1
   rgb[{{},{near_h,near_h+1},{near_w,far_w}}] = 1
   rgb[{{},{near_h,far_h},{far_w,far_w+1}}] = 1
   rgb[{{},{far_h,far_h+1},{near_w,far_w}}] = 1
end

function pf.remove_plane(rgb, mask)      -- removed pts black
   rgb[1][mask] = 0
   rgb[2][mask] = 0
   rgb[3][mask] = 0
end

-- DxWxH to NxD with mask
function pf.get_points_from_map(map,mask,scratch)
   local pts
   local npts = map[1]:nElement()
   if mask then
      npts    = mask:sum()
      scratch = scratch or torch.Tensor()
      pts     = scratch:resize(3,npts)
      pts[1]  = map[1][mask]
      pts[2]  = map[2][mask]
      pts[3]  = map[3][mask]
   else
      pts = map:reshape(map:size(1),map:size(2)*map:size(3))
   end
   return pts:t():contiguous(),npts
end

function pf.newErode(erosion_amount,image_h, image_w)
   erosion  = image.newErosion(erosion_amount)
   return function (mask)
      -- printf("eroding: %d",mask:sum())
      mask = mask:double():resize(image_h,image_w)
      mask = erosion(mask):gt(0)
      -- printf(" -> to %d",mask:sum())
      return mask
   end
end

function pf.newErodeDilate(erosion_amount,dilation_amount,smoothing_amount,image_h, image_w)
   erosion  = image.newErosion(erosion_amount)
   dilation = image.newDilation(dilation_amount,smoothing_amount)
   return function (mask)
      printf("eroding: %d",mask:sum())
      mask = mask:double():resize(image_h,image_w)
      local mer = erosion(mask)
      local mdl = dilation(mer)
      mask = mdl:gt(0)
      printf(" -> to %d",mask:sum())
      return mask
   end
end

function pf.select_points(pts,mask)
   local npts   = mask:sum()
   local mask   = mask:reshape(mask:nElement(),1):expand(mask:nElement(),3)
   local outpts = pts[mask]
   return outpts:resize(npts,3), npts
end

function pf.get_patch_pts(idx,win_h,win_w,data,mask)
   local dims  = data:nDimension()
   local img_w = data:size(dims)
   local img_h = data:size(dims-1)
   local bbx   = compute_bbx(idx,win_h, win_w, img_h, img_w)
   local win   = data[{{},{bbx[1],bbx[2]},{bbx[3],bbx[4]}}]
   if mask then
      mask = mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}]
   end
   return get_points_from_map(win,mask)
end

function pf.find_points(points,plane_eq,threshold,erodeDilate)
   -- points Nx3
   local res      = compute_residual(points,plane_eq)
   local mask     = res:abs():lt(threshold)
   if erodeDilate then 
      mask = erodeDilate(mask)
   end
   local pts,npts = select_points(points,mask)
   return pts, npts, mask
end

function pf.score(points, plane_eq)
   local res = compute_residual(points,plane_eq)
   return res:std()
end

-- TODO in C ackkkkk, need a generic bytemap to 1D index and 1D index to ND index
function pf.get_vals_index(bmap,map)
   local k = bmap:sum()
   local idx = torch.LongTensor(2,k)
   local val = torch.Tensor(k)
   local c = 0
   for i = 1,bmap:size(1) do
      for j = 1,bmap:size(2) do
         if mx[i][j] > 0 then
            c = c + 1
            idx[1][c] = i
            idx[2][c] = j
            val[c] = mx[i][j]
         end
      end
   end
   return idx, val, c
end
function pf.merge_planes(points,threshold,p1,p2,erodeDilate)
   -- compute overlap between points in each plane
   local intersection_mask = torch.cmul(p1.mask,p2.mask)
   local n_intersection    = intersection_mask:sum()
   local union_mask        = torch.add(p1.mask,p2.mask):gt(0)
   local n_union           = union_mask:sum()
   local points_to_beat    = math.max(p1.npts, p2.npts)
   local score_to_beat     = math.min(p1.score, p2.score)
   if n_intersection > 0.1 * n_union then
      -- points in union and intersect
      printf(" - testing %d intersection %d union",n_intersection,n_union)
      printf("  - max of old 2 planes explains: %d points score: %f %f", points_to_beat, p1.score, p2.score)
      union_pts, union_npts = select_points(points,union_mask)
      inter_pts, inter_npts = select_points(points,intersection_mask)

      -- fit planes to union and intersect
      union_plane_eq = fit_plane(union_pts)
      inter_plane_eq = fit_plane(inter_pts)

      -- residual w/respect to points in set (must be low or set has outliers)
      union_score = score(union_pts, union_plane_eq)
      inter_score = score(inter_pts, inter_plane_eq)

      local new_union_pts,new_union_npts,new_union_plane_pts_mask,new_union_score
      -- find set of points (from all points) close to the new planes and score
      if (union_score < threshold) then
         new_union_pts,new_union_npts,new_union_plane_pts_mask =
            find_points(points,union_plane_eq,threshold,erodeDilate)
         new_union_score = score(new_union_pts, union_plane_eq)
         printf("  - plane fit on union explains:       %d points score %f was %f", 
                new_union_npts, new_union_score, union_score)
      end
      local new_inter_pts,new_inter_npts,new_inter_plane_pts_mask, new_inter_score
      if (inter_score < threshold) then
         new_inter_pts,new_inter_npts,new_inter_plane_pts_mask =
            find_points(points,inter_plane_eq,threshold,erodeDilate)
         new_inter_score = score(new_inter_pts, inter_plane_eq)
         printf("  - plane fit on intersection explains: %d points score: %f was %f", 
                new_inter_npts, new_inter_score, inter_score)
      end

      -- with new plane vs. %d with combined old", new_npts, npts)
      if new_union_pts and (new_union_score < threshold) and (
         (new_union_npts > points_to_beat) or (new_union_score < score_to_beat)) then
         printf("  - combining with plane fit on union")
         mask = new_union_plane_pts_mask:resizeAs(p1.mask)
         -- return merged
         return {eq       = union_plane_eq,
                 npts     = new_union_npts,
                 score    = new_union_score,
                 mask     = mask,
                 win_idx  = table.cat(p1.win_idx,p2.win_idx),
                 win_size = table.cat(p1.win_size,p2.win_size)
         }
      elseif new_inter_pts and (new_inter_score < threshold) and (
         (new_inter_npts > points_to_beat) or ((new_inter_score < score_to_beat)))
      then
         printf("  - combining with plane fit on intersection")
         mask = new_inter_plane_pts_mask:resizeAs(p1.mask)
         -- return merged
         return {eq       = inter_plane_eq,
                 npts     = new_inter_npts,
                 score    = new_inter_score,
                 mask     = mask,
                 win_idx  = table.cat(p1.win_idx,p2.win_idx),
                 win_size = table.cat(p1.win_size,p2.win_size)
         }
      end
   end
end

-- loops through planes in input_planes, combines them if the
-- intersection is sufficient, and score of combined plane is better.
function pf.combine_planes (points,input_planes,threshold,erode_dilate)
   local n_input_planes = #input_planes
   printf("** Combining %d planes with threshold: %f",n_input_planes, threshold)
   local removed        = {}
   local output_planes  = {}
   for i = 1,n_input_planes-1 do
      if not removed[i] then
         p1      = input_planes[i]
         for j = i+1,n_input_planes do
            if not removed[j] then
               p2      = input_planes[j]
               p3 = merge_planes(points,threshold,p1,p2,erode_dilate)
               if p3 then
                  p1 = p3 -- update the current plane
                  removed[j] = true
               end
            end
         end
         -- if not removed add possibly updated p1 to output
         table.insert(output_planes, p1)
      end
   end
   -- make sure we add the last one
   if not removed[n_input_planes] then
      table.insert(output_planes, input_planes[n_input_planes])
   end
   return output_planes
end

function pf.add_planes(points, planes, candidate_planes, threshold, erode_dilate)
   local output_planes = {}
   -- copy the planes
   for i,p in pairs(planes) do
      table.insert(output_planes,p)
   end
   -- attempt to add candidates to existing planes
   local n_candidate_planes = #candidate_planes

   printf("** Adding %d planes to %d with threshold: %f",n_candidate_planes, #output_planes, threshold)
   for i = 1,n_candidate_planes do
      local p1       = candidate_planes[i]
      local n_planes = #output_planes
      -- try to add planes to existing planes or create new planes
      local not_merged = true
      local j = 1
      while not_merged and (j < n_planes) do
         local p2 = output_planes[j]
         local p3 = merge_planes(points,threshold,p1,p2,erode_dilate)
         -- break (TODO check if greedy merge with first matching plane is the best strategy)
         if p3 then
            -- replace output plane with new merged
            output_planes[j] = p3
            not_merged = false
         end
         j = j + 1
      end -- check about adding to existing planes
      if not_merged then
         -- add candidate
         table.insert(output_planes, candidate_planes[i])
         printf("  - adding candidate as new plane %d",#planes)
      end
   end
   return output_planes
end

function pf.recompute_valid_points(valid_points,planes)
   local vpts = valid_points:clone()
   for i = 1,#planes do
      vpts:cmul(planes[i].mask:eq(0))
   end
   return vpts
end


-- keep colors consistent
local colors = torch.rand(1000,3)

function pf.visualize_planes(planes, rgb)
   rgb = rgb or torch.Tensor(3,planes[1].mask:size(1),planes[1].mask:size(2)):fill(0)

   for i = 1,#planes do
      pl = planes[i]
      m = pl.mask
      rgb[1][m] = colors[i][1]
      rgb[2][m] = colors[i][2]
      rgb[3][m] = colors[i][3]

      for j,p in pairs(pl.win_idx) do
         if pl.win_size and pl.win_size[j] then
            wx = pl.win_size[j].x
            wy = pl.win_size[j].y
            -- visualize saliency window white
            draw_window(rgb,p,wx,wy)
         else
            print("something wrong with ",pl)
         end
      end
   end
   image.display(rgb)
   return rgb
end

return pf
