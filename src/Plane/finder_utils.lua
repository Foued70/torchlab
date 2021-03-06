local pi               = math.pi
local linear_plane     = geom.linear_model.fit
local compute_residual = geom.linear_model.residual

local fu = Class()

-- same as geom.linear_model.fit but flips the normal to point towards the origin
function fu.fit_plane(pts)
   local plane_eqn = linear_plane(pts)
   if plane_eqn[4] > 0 then 
      plane_eqn:mul(-1)
   end
   return plane_eqn
end

function fu.compute_bbx(idx,win_h,win_w,img_h,img_w)
   local hwin_h  = win_h / 2
   local hwin_w  = win_w / 2
   local near_h  = math.max(1,idx[1]-hwin_h)
   local far_h   = math.min(img_h-hwin_h,near_h+win_h)
   local near_w  = math.max(1,idx[2]-hwin_w)
   local far_w   = math.min(img_w-hwin_w,near_w+win_w)
   return {near_h, far_h, near_w, far_w }
end

function fu.draw_window(rgb,idx,win_w,win_h)
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

function fu.remove_plane(rgb, mask)      -- removed pts black
   rgb[1][mask] = 0
   rgb[2][mask] = 0
   rgb[3][mask] = 0
end

-- DxWxH to NxD with mask
function fu.get_points_from_map(map,mask,scratch)
   local pts
   local n_pts = map[1]:nElement()
   if mask then
      n_pts   = mask:sum()
      scratch = scratch or torch.Tensor()
      pts     = scratch:resize(3,n_pts)
      pts[1]  = map[1][mask]
      pts[2]  = map[2][mask]
      pts[3]  = map[3][mask]
   else
      pts = map:reshape(map:size(1),map:size(2)*map:size(3))
   end
   return pts:t():contiguous(),n_pts
end

function fu.newErode(erosion_amount,image_h, image_w)
   erosion  = image.newErosion(erosion_amount)
   return function (mask)
      mask = mask:double():resize(image_h,image_w)
      mask = erosion(mask):gt(0)
      return mask
   end
end

function fu.newErodeDilate(erosion_amount,dilation_amount,smoothing_amount,image_h, image_w)
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

function fu.get_patch_pts(idx,win_h,win_w,data,mask)
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


-- TODO in C ackkkkk, need a generic bytemap to 1D index and 1D index to ND index
function fu.get_vals_index(bmap,map)
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
            val[c] = map[i][j]
         end
      end
   end
   return idx, val, c
end

function fu.get_max_val_index(map)
   local v2,idx2 = map:max(1):max(2)
   local v1,idx1 = map:max(2):max(1)
   v1 = v1:squeeze()
   v2 = v2:squeeze()
   idx1 = idx1:squeeze()
   idx2 = idx2:squeeze()
   if v1 ~= v2 then 
      print(idx1, idx2, v1, v2)
      error("something is very wrong") end
   return torch.Tensor({idx1,idx2}), v1
end

function fu.get_mask(points, plane_eqn, threshold, normal_filter, normal_threshold, normals)
   local pts, n_pts, mask =
      find_points_explained_by_plane(points, plane_eqn, threshold, 
                                     normal_filter, normal_threshold, normals)
   return mask
end

function fu.merge_planes(points,threshold,p1,p2,intersection_threshold,normal_filter,normal_threshold,normals)
   -- compute overlap between points in each plane
   p1.mask = p1.mask or get_mask(points,p1.eqn,threshold,normal_filter,normal_threshold,normals)
   p2.mask = p2.mask or get_mask(points,p2.eqn,threshold,normal_filter,normal_threshold,normals)

   -- if (p1.mask:sum() ~= p1.n_pts) then 
   --    log.trace("new thres: ", p1.mask:sum(),"old: ", p1.n_pts)
   --    -- error("recreating points explained by plane")
   -- end

   -- if (p2.mask:sum() ~= p2.n_pts) then 
   --    log.trace("new thres: ", p2.mask:sum(),"old: ", p2.n_pts)
   --    -- error("recreating points explained by plane")
   -- end

   local intersection_mask = torch.cmul(p1.mask,p2.mask)
   local n_intersection    = intersection_mask:sum()
   local union_mask        = torch.add(p1.mask,p2.mask):gt(0)
   local n_union           = union_mask:sum()
   local p_bigger          = p1.n_pts >= p2.n_pts and p1 or p2
   local p_smaller         = p1.n_pts <  p2.n_pts and p1 or p2
   
   local p_merge      = nil
   local combo_string = "not combined"
   local score_data  = { large_plane_npts  = p_bigger.n_pts, 
                         large_plane_score = p_bigger.score, 
                         small_plane_npts  = p_smaller.n_pts, 
                         small_plane_score = p_smaller.score,
                         n_intersection    = n_intersection,
                         n_union           = n_union
   }


   -- test 1: no intersection return nil
   -- the intersection must cover some percent of the smaller patch
   if n_intersection < intersection_threshold * p_smaller.n_pts then 
      combo_string = "not combining: no intersection"
      return nil, combo_string, score_data 
   end

   -- test 2: total intersection return bigger (TODO: could check normal and "carve" into larger plane)
   if n_intersection > (1 - intersection_threshold) * p_smaller.n_pts then 
      combo_string = "combining larger subsumes smaller"
      return p_bigger , combo_string, score_data
   end

   -- test 3: plane normals beyond threshold
   local norm_diff = p1.eqn[{{1,3}}] - p2.eqn[{{1,3}}]
   local norm_dist = math.sqrt(norm_diff:cmul(norm_diff):sum())
   if norm_dist > normal_threshold then 
      combo_string = "not combining: normals beyond threshold"
      return nil, combo_string, score_data 
   end

   --test 4: harder tests about fitting planes on different sets
   -- points in union and intersect
   -- print(score_data)

   local union_pts, union_n_pts = select_points(points,union_mask)
   local inter_pts, inter_n_pts = select_points(points,intersection_mask)

   -- fit planes to union and intersect
   local union_plane_eqn = fit_plane(union_pts)
   local inter_plane_eqn = fit_plane(inter_pts)

   -- residual w/respect to points in set (must be low or set has outliers)
   local union_score = score(union_plane_eqn, union_pts)
   local inter_score = score(inter_plane_eqn, inter_pts)

   -- TODO no locals just store everying in table 
   score_data.union_n_pts     = union_n_pts
   score_data.union_score     = union_score
   score_data.union_plane_eqn = union_plane_eqn
   score_data.inter_n_pts     = inter_n_pts
   score_data.inter_score     = inter_score
   score_data.inter_plane_eqn = inter_plane_eqn

   local new_union_pts,new_union_n_pts,new_union_plane_pts_mask,new_union_score
   -- find set of points (from all points) close to the new planes and score
   if (union_score < threshold) then
      new_union_pts,new_union_n_pts, new_union_plane_pts_mask =
         find_points_explained_by_plane(points,union_plane_eqn,threshold, 
                                        normal_filter, normal_threshold, normals)
      new_union_score = score(union_plane_eqn, new_union_pts)
      printf("  - plane fit on union is good %f and fits %d with score %f:",
             union_score, new_union_n_pts, new_union_score )
      score_data.exp_union_score = new_union_score
      score_data.exp_union_n_pts = new_union_n_pts
   end

   local new_inter_pts,new_inter_n_pts,new_inter_plane_pts_mask, new_inter_score
   if (inter_score < threshold) then
      new_inter_pts,new_inter_n_pts, new_inter_plane_pts_mask = 
         find_points_explained_by_plane(points, inter_plane_eqn, threshold, 
                                        normal_filter, normal_threshold, normals)
      new_inter_score = score(inter_plane_eqn, new_inter_pts)
      printf("  - plane fit on inter is good %f and fits %d with score %f:",
             inter_score, new_inter_n_pts, new_inter_score)
      score_data.exp_inter_score = new_union_score
      score_data.exp_inter_n_pts = new_union_n_pts
   end
   
   if (union_score < inter_score)  then
      -- if score of plane fit to the union is lower then prefer the union
      if new_union_pts and (new_union_score < threshold) and (new_union_n_pts > p_bigger.n_pts) then
         printf("  * combining with plane fit on union b/c union_score lower")
         local mask = new_union_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = union_plane_eqn,
                    n_pts    = new_union_n_pts,
                    score    = new_union_score,
                    mask     = mask
         }
         combo_string = "inter b/c inter_score is lower"
      elseif new_inter_pts and (new_inter_score < threshold) and (new_inter_n_pts > p_bigger.n_pts) then
         printf("  * combining with plane fit on intersection though union_score lower")
         local mask = new_inter_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = inter_plane_eqn,
                    n_pts    = new_inter_n_pts,
                    score    = new_inter_score,
                    mask     = mask
         }
         combo_string = "inter though inter_score is lower"
      end
   else
      -- else score of intersection is lower and prefer intersection
      if new_inter_pts and (new_inter_score < threshold) and (new_inter_n_pts > p_bigger.n_pts) then
         printf("  * combining with plane fit on intersection b/c inter_score is lower")
         local mask = new_inter_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = inter_plane_eqn,
                    n_pts    = new_inter_n_pts,
                    score    = new_inter_score,
                    mask     = mask
         }
         combo_string = "inter b/c inter_score is lower"
      elseif new_union_pts and (new_union_score < threshold) and (new_union_n_pts > p_bigger.n_pts) then
         printf("  * combining with plane fit on union")
         local mask = new_union_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = union_plane_eqn,
                    n_pts    = new_union_n_pts,
                    score    = new_union_score,
                    mask     = mask
         }
         combo_string = "union though inter_score is lower"
      end 
   end

   if p_merge then
      if p1.debug_info and p2.debug_info and
      (type(p1.debug_info) == "table") and (type(p2.debug_info) == "table") then
         p_merge.debug_info  = table.cat(p1.debug_info,p2.debug_info)
      end
   end

   return p_merge, combo_string, score_data
end

function add_to_score(score_hash, score_str, score_data, i, j)
   score_hash = score_hash or {}
   score_data = score_data or {}
   score_data.from = i 
   score_data.to   = j
   if score_hash[score_str] then 
      hash_bucket = score_hash[score_str]
      hash_bucket.cnt = hash_bucket.cnt + 1
      if hash_bucket.data then
         table.insert(hash_bucket.data, score_data)
      end
   else
      score_hash[score_str] =  {}
      hash_bucket = score_hash[score_str]
      hash_bucket.cnt = 1
      if score_data then
         hash_bucket.data = { score_data}
      end
   end
   return score_hash
end

-- loops through planes in input_planes, combines them if the
-- intersection is sufficient, and score of combined plane is better.
function fu.combine_planes (points,input_planes,threshold,
                            normal_filter, normal_threshold, normals,
                            score_keeper, loser_keeper)
   local n_input_planes = #input_planes
   printf("** Combining %d planes with threshold: %f",n_input_planes, threshold)
   local removed        = {}
   local output_planes  = {input_planes[1]}
   for i = 2,n_input_planes do
      local p1         = input_planes[i]
      local n_planes   = #output_planes
      local not_merged = true
      local j = 1
      while not_merged and (j <= n_planes) do 
         -- printf(" ++ testing %d to %d",i,j)
         local p2  = output_planes[j]
         local p3,str,data = 
            merge_planes(points,threshold,p1,p2,0.1,normal_filter, normal_threshold, normals)
         if p3 then
            output_planes[j] = p3
            not_merged = false
            -- print("merged "..str)
            add_to_score(score_keeper, str, data, i, j)
         else
            -- print("not merging "..str)
            add_to_score(loser_keeper, str, data, i, j)
         end
         j = j + 1
      end
      if not_merged then
         -- print("not merged STOP")
         -- if not removed add p1 to output
         table.insert(output_planes, p1)
      end
      collectgarbage()
   end
   return output_planes, score_keeper
end

function fu.add_planes(points, planes, candidate_planes, threshold, 
                       normal_filter, normal_threshold, normals, 
                       score_keeper)
   -- attempt to add candidates to existing planes
   local n_candidate_planes = #candidate_planes

   printf("** Adding %d planes to %d with threshold: %f",n_candidate_planes, #planes, threshold)
   for i = 1,n_candidate_planes do
      local p1       = candidate_planes[i]
      local n_planes = #planes
      -- try to add planes to existing planes or create new planes
      local not_merged = true
      local j = 1
      while not_merged and (j < n_planes) do
         local p2 = planes[j]
         local p3,str,data = merge_planes(points,threshold,p1,p2,0.1,normal_filter, normal_threshold, normals)
         -- break (TODO check if greedy merge with first matching plane is the best strategy)
         if p3 then
            -- replace output plane with new merged
            planes[j] = p3
            not_merged = false
            score_keeper = add_to_score(score_keeper, str, data)
         end
         j = j + 1
      end -- check about adding to existing planes
      if not_merged then
         -- add candidate
         table.insert(planes, candidate_planes[i])
         printf("  - adding candidate as new plane %d",#planes)
      end
   end
   return planes, score_keeper
end

function fu.recompute_valid_points(valid_points,planes)
   local vpts = valid_points:clone()
   for i = 1,#planes do
      vpts:cmul(planes[i].mask:eq(0))
   end
   return vpts
end


-- keep colors consistent
local colors = image.colormap(1000)

function fu.visualize_planes(planes, height, width, rgb)
   local pmask = planes[1].mask
   height = height or pmask:size(1)
   width = width or pmask:size(2)
   rgb = rgb or torch.Tensor(3,height,width):fill(0)

   for i = #planes,1,-1 do
      local pl = planes[i]
      local m = pl.mask
      if not m then 
         print("warning no mask for plane "..i)
      else
         rgb[1][m] = colors[i][1]
         rgb[2][m] = colors[i][2]
         rgb[3][m] = colors[i][3]
         
         if pl.debug_info then
            for j,p in pairs(pl.debug_info) do
               if (p.win_size and p.win_idx) then
                  local wx = p.win_size.x
                  local wy = p.win_size.y
                  -- visualize saliency window white
                  draw_window(rgb,p.win_idx,wx,wy)
               end
            end
         end
      end
   end
   return image.combine(rgb)
end

function fu.filter_by_normal(plane_eqn, mask, normal_threshold, normals)
   local ndim = normals:size(normals:nDimension())
   local explained_normals =
      select_points(normals,mask)
   local diff =
      (explained_normals - plane_eqn[{{1,ndim}}]:resize(1,ndim):expandAs(explained_normals))
   local filtered_mask = mask:clone()   
   filtered_mask[mask] =
      diff:cmul(diff):sum(2):squeeze():sqrt():lt(normal_threshold)
   return filtered_mask
end


function fu.find_rotation_to_align_axis(norm)
   local angle_az = geom.util.unit_cartesian_to_spherical_angles(norm)
   -- TODO check this again.  Disconnect btw. 2D projections and 3D.  I would have expected azimuth in first
   return geom.quaternion.from_euler_angle(torch.Tensor({0,angle_az[1]}))
end

function fu.axis_align(pts,quat)
   quat = quat or find_rotation_to_align_axis(pts)
   return geom.quaternion.rotate(quat,pts), quat
end

function fu.dump_pts(pts,fname)
   fname = fname or "points.xyz"
   io = require 'io'
   f = io.open(fname,'w')
   i = 1
   pts:apply(function (x)
                f:write(string.format("%f ",x))
                if (i%3 == 0) then f:write("\n") end
                i = i+1
             end)
   f:close()
end

-- align
function fu.align_planes(_planes,_allpts)
   local maxv = -1
   local midx = 1
   for i,p in pairs(_planes) do
      if (p.eqn[1] > maxv) and (p.n_pts > _allpts:size(1)*0.01) then
         maxv = p.eqn[1] ;
         midx = i ;
      end
   end

   local pm    = _planes[midx]
   local quat  = find_rotation_to_align_axis(pm.eqn)
   local aapts = axis_align(_allpts,quat)
   return quat, aapts
end

function fu.dump_pts_to_xyz (planes, aapts, fname)
   for i,p in pairs(planes) do
      ppts = select_points(aapts,p.mask)
      dump_pts(ppts,string.format("%s_%d_pts.xyz",fname,i))
   end
end

function fu.aligned_planes_to_obj (planes, aapts, quat, fname)
   io = require 'io'
   fname = fname or "planes"

   local objf = io.open(fname..".obj",'w')

   -- loop through planes. rotate normals.  find bbx in plane. create rectangle.
   for i,p in pairs(planes) do

      local ppts = select_points(aapts,p.mask)
      
      local pn = geom.quaternion.rotate(quat,p.eqn)
      local pcntr = ppts:mean(1):squeeze()
      pn[4] = -pn[{{1,3}}] * pcntr
      local pmin = ppts:min(1):squeeze()
      local pmax = ppts:max(1):squeeze()
      local s = torch.sign(pn)
      local m = torch.abs(pn[{{1,3}}])
      local _,mi = m:max(1)
      mi = mi:squeeze()

      if ((mi == 1) and (s[mi] > 0)) then
         -- if +x then
         for _,bbx in pairs({{pmin[2],pmax[3]},{pmin[2],pmin[3]},{pmax[2],pmin[3]},{pmax[2],pmax[3]}}) do
            objf:write(string.format("v %d %d %d\n",-(bbx[1]*pn[2] + bbx[2]*pn[3] + pn[4])/pn[1],bbx[1],bbx[2]))
         end
      elseif ((mi == 1) and (s[mi] < 0)) then
         -- if -x then
         for _,bbx in pairs({{pmin[2],pmax[3]},{pmax[2],pmax[3]},{pmax[2],pmin[3]},{pmin[2],pmin[3]}}) do
            objf:write(string.format("v %d %d %d\n",-(bbx[1]*pn[2] + bbx[2]*pn[3] + pn[4])/pn[1],bbx[1],bbx[2]))
         end
      elseif ((mi == 2) and (s[mi] > 0)) then
         -- if +y then
         for _,bbx in pairs({{pmin[1],pmax[3]},{pmin[1],pmin[3]},{pmax[1],pmin[3]},{pmax[1],pmax[3]}}) do
            objf:write(string.format("v %d %d %d\n",bbx[1],-(bbx[1]*pn[1] + bbx[2]*pn[3] + pn[4])/pn[2],bbx[2]))
         end
      elseif ((mi == 2) and (s[mi] < 0)) then
         -- if -y then
         for _,bbx in pairs({{pmin[1],pmax[3]},{pmax[1],pmax[3]},{pmax[1],pmin[3]},{pmin[1],pmin[3]}}) do
            objf:write(string.format("v %d %d %d\n",bbx[1],-(bbx[1]*pn[1] + bbx[2]*pn[3] + pn[4])/pn[2],bbx[2]))
         end
      elseif ((mi == 3) and (s[mi] > 0)) then
         -- if +z then
         for _,bbx in pairs({{pmin[1],pmax[2]},{pmin[1],pmin[2]},{pmax[1],pmin[2]},{pmax[1],pmax[2]}}) do
            objf:write(string.format("v %d %d %d\n",bbx[1],bbx[2],-(bbx[1]*pn[1] + bbx[2]*pn[2] + pn[4])/pn[3]))
         end
      elseif ((mi == 3) and (s[mi] < 0)) then
         -- if -z then
         for _,bbx in pairs({{pmin[1],pmax[2]},{pmax[1],pmax[2]},{pmax[1],pmin[2]},{pmin[1],pmin[2]}}) do
            objf:write(string.format("v %d %d %d\n",bbx[1],bbx[2],-(bbx[1]*pn[1] + bbx[2]*pn[2] + pn[4])/pn[3]))
         end
      end
   end

   -- write faces
   local s = "f "
   for i = 1,#planes*4 do
      s = string.format("%s %d",s,i)
      if (i%4) == 0 then
         objf:write(s.."\n")
         s = "f "
      end
   end
   objf:close()
end

return fu
