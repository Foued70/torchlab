pi               = math.pi
linear_plane     = geom.linear_model.fit
compute_residual = geom.linear_model.residual

pf = Class()

-- same as geom.linear_model.fit but flips the normal to point towards the origin
function pf.fit_plane(pts)
   local plane_eqn = linear_plane(pts)
   if plane_eqn[4] > 0 then 
      plane_eqn:mul(-1)
   end
   return plane_eqn
end

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
   local ptsd   = pts:size(pts:nDimension())
   local npts   = mask:sum()
   local mask   = mask:reshape(mask:nElement(),1):expand(mask:nElement(),ptsd)
   local outpts = pts[mask]
   return outpts:resize(npts,ptsd), npts
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

function pf.find_points_mask(points,plane_eqn,threshold)
   -- points Nx3
   return compute_residual(points,plane_eqn):abs():lt(threshold)
end

function pf.find_points_explained_by_plane(points, plane_eqn, threshold, normal_filter, normal_threshold, normals)
   plane_pts_mask = find_points_mask(points,plane_eqn,threshold)
   if normal_filter then 
      plane_pts_mask =
         filter_by_normal(plane_eqn, plane_pts_mask, normal_threshold, normals)
   end
   pts,npts = select_points(points, plane_pts_mask)
   return pts, npts, plane_pts_mask
end

function pf.score(points, plane_eqn)
   return compute_residual(points,plane_eqn):std()
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

function pf.merge_planes(points,threshold,p1,p2,intersection_threshold,normal_filter,normal_threshold,normals)
   -- compute overlap between points in each plane
   local intersection_mask = torch.cmul(p1.mask,p2.mask)
   local n_intersection    = intersection_mask:sum()
   local union_mask        = torch.add(p1.mask,p2.mask):gt(0)
   local n_union           = union_mask:sum()
   local p_bigger          = p1.npts >= p2.npts and p1 or p2
   local p_smaller         = p1.npts <  p2.npts and p1 or p2
   
   local p_merge = nil
   local combo_string = "not combined"
   local score_data = 
      string.format("large_plane: %d %f small_plane: %d %f ",
                    p_bigger.npts, p_bigger.score, p_smaller.npts, p_smaller.score)

   -- test 1: no intersection return nil
   -- the intersection must cover some percent of the smaller patch
   if n_intersection < intersection_threshold * p_smaller.npts then 
      combo_string = "not combining: no intersection"
      print(" ** " .. combo_string)
      print("    " .. score_data)
      return nil, combo_string, score_data 
   end

   -- test 2: total intersection return bigger (TODO: could check normal and "carve" into larger plane)
   if n_intersection > (1 - intersection_threshold) * p_smaller.npts then 
      combo_string = "combining larger subsumes smaller"
      print(" ** " .. combo_string)
      print("    " .. score_data)
      return p_bigger , combo_string, score_data
   end

   -- test 3: plane normals beyond threshold
   norm_diff = p1.eqn[{{1,3}}] - p2.eqn[{{1,3}}]
   norm_dist = math.sqrt(norm_diff:cmul(norm_diff):sum())
   if norm_dist > normal_threshold then 
      combo_string = "not combining: normals beyond threshold"
      print(" ** " .. combo_string)
      print("    " .. score_data)
      return nil, combo_string, score_data 
   end

   --test 4: harder tests about fitting planes on different sets
   -- points in union and intersect
   printf("  + testing merge npts: %d intersection %d union bigger patch : %d smaller: %d",
          n_intersection, n_union, p_bigger.npts, p_smaller.npts)
   printf("  - %s", score_data)

   local union_pts, union_npts = select_points(points,union_mask)
   local inter_pts, inter_npts = select_points(points,intersection_mask)

   -- fit planes to union and intersect
   local union_plane_eqn = fit_plane(union_pts)
   local inter_plane_eqn = fit_plane(inter_pts)

   -- residual w/respect to points in set (must be low or set has outliers)
   local union_score = score(union_pts, union_plane_eqn)
   local inter_score = score(inter_pts, inter_plane_eqn)

   local new_union_pts,new_union_npts,new_union_plane_pts_mask,new_union_score
   -- find set of points (from all points) close to the new planes and score
   if (union_score < threshold) then
      new_union_pts,new_union_npts, new_union_plane_pts_mask =
         find_points_explained_by_plane(points,union_plane_eqn,threshold, 
                                        normal_filter, normal_threshold, normals)
      new_union_score = score(new_union_pts, union_plane_eqn)
      printf("  - plane fit on union is good %f and fits %d with score %f:",
             union_score, new_union_npts, new_union_score )
      score_data = string.format("%s union: %f %d %f",
                                 score_data, union_score, new_union_npts, new_union_score)
   end

   local new_inter_pts,new_inter_npts,new_inter_plane_pts_mask, new_inter_score
   if (inter_score < threshold) then
      new_inter_pts,new_inter_npts, new_inter_plane_pts_mask = 
         find_points_explained_by_plane(points, inter_plane_eqn, threshold, 
                                        normal_filter, normal_threshold, normals)
      new_inter_score = score(new_inter_pts, inter_plane_eqn)
      printf("  - plane fit on inter is good %f and fits %d with score %f:",
             inter_score, new_inter_npts, new_inter_score)
      score_data = string.format("%s intersection: %f %d %f", 
                                 score_data, inter_score, new_inter_npts, new_inter_score)
   end
   
   if (union_score < inter_score)  then
      -- if score of plane fit to the union is lower then prefer the union
      if new_union_pts and (new_union_score < threshold) and (new_union_npts > p_bigger.npts) then
         printf("  * combining with plane fit on union b/c union_score lower")
         local mask = new_union_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = union_plane_eqn,
                    npts     = new_union_npts,
                    score    = new_union_score,
                    mask     = mask
         }
         combo_string = "inter b/c inter_score is lower"
      elseif new_inter_pts and (new_inter_score < threshold) and (new_inter_npts > p_bigger.npts) then
         printf("  * combining with plane fit on intersection though union_score lower")
         local mask = new_inter_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = inter_plane_eqn,
                    npts     = new_inter_npts,
                    score    = new_inter_score,
                    mask     = mask
         }
         combo_string = "inter though inter_score is lower"
      end
   else
      -- else score of intersection is lower and prefer intersection
      if new_inter_pts and (new_inter_score < threshold) and (new_inter_npts > p_bigger.npts) then
         printf("  * combining with plane fit on intersection b/c inter_score is lower")
         local mask = new_inter_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = inter_plane_eqn,
                    npts     = new_inter_npts,
                    score    = new_inter_score,
                    mask     = mask
         }
         combo_string = "inter b/c inter_score is lower"
      elseif new_union_pts and (new_union_score < threshold) and (new_union_npts > p_bigger.npts) then
         printf("  * combining with plane fit on union")
         local mask = new_union_plane_pts_mask:resizeAs(p_bigger.mask)
         -- return merged
         p_merge = {eqn      = union_plane_eqn,
                    npts     = new_union_npts,
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

   print(" ** " .. combo_string)
   print("    " .. score_data)
   return p_merge, combo_string, score_data
end

function add_to_score(score_hash, score_str, score_data)
   score_hash = score_hash or {}
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
function pf.combine_planes (points,input_planes,offset,threshold,
                            normal_filter, normal_threshold, normals,
                            score_keeper)
   local n_input_planes = #input_planes
   printf("** Combining %d planes with threshold: %f",n_input_planes, threshold)
   local removed        = {}
   local output_planes  = {}
   for i = offset,n_input_planes-1 do
      if not removed[i] then
         p1      = input_planes[i]
         for j = i+1,n_input_planes do
            if not removed[j] then
               p2  = input_planes[j]
               p3,str,data  = merge_planes(points,threshold,p1,p2,0.5,normal_filter, normal_threshold, normals)
               if p3 then
                  p1 = p3 -- update the current plane
                  removed[j] = true
                  score_keeper = add_to_score(score_keeper, str, data)
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
   return output_planes, score_keeper
end

function pf.add_planes(points, planes, candidate_planes, threshold, 
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

function pf.recompute_valid_points(valid_points,planes)
   local vpts = valid_points:clone()
   for i = 1,#planes do
      vpts:cmul(planes[i].mask:eq(0))
   end
   return vpts
end


-- keep colors consistent
torch.manualSeed(1)
local colors = torch.rand(1000,3)
torch.seed() -- make things random again

function pf.visualize_planes(planes, rgb)
   rgb = rgb or torch.Tensor(3,planes[1].mask:size(1),planes[1].mask:size(2)):fill(0)

   for i = #planes,1,-1 do
      pl = planes[i]
      m = pl.mask
      rgb[1][m] = colors[i][1]
      rgb[2][m] = colors[i][2]
      rgb[3][m] = colors[i][3]

      if pl.debug_info then
         for j,p in pairs(pl.debug_info) do
            if (p.win_size and p.win_idx) then
               wx = p.win_size.x
               wy = p.win_size.y
               -- visualize saliency window white
               draw_window(rgb,p.win_idx,wx,wy)
            else
               print("something wrong with ",pl)
            end
         end
      end
   end
   return image.combine(rgb)
end

function pf.filter_by_normal(plane_eqn, mask, normal_threshold, normals)
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

-- function plane from seed
-- output: plane or error_flag
dbg_count = 0
function pf.from_seed(...)
   local _, points, mask, threshold, min_points_for_seed, min_points_for_plane,
            expanded_set, 
            normal_filter, normal_threshold, normals, 
            erosion_filter, erosion_function,
            debug_info = 
               dok.unpack(
      {...},
      'plane_finder.from_seed',
      'validate a plane in a set of points given a subset of points',
      {arg='points',
       type='torch.Tensor',
       help='Nx3 list of points corresponding to mask',
       req=true},
      {arg='mask',
       type='torch.ByteTensor',
       help='(HxW or N) 1 where points are valid 0 elsewhere',
       req=true},
      {arg='threshold',
       type='number',
       help='minimum score (std of residual) to accept a plane',
       req=true},
      {arg='min_points_for_seed',
       type='number',
       help='minimum number of points to consider set at seed',
       default=150},
      {arg='min_points_for_plane',
       type='number',
       help='minimum number of points to consider set a plane',
       default=900},
      {arg='expanded_set',
       type="boolean",
       help="should we try to find better plane on larger set",
       default=false},
      {arg='normal_filter',
       type="boolean",
       help="should we try to find better plane on larger set",
       default=false},
      {arg='normal_threshold',
       type='number',
       help='minimum difference in point normals to accept as part of same plane',
      default=pi/6},
      {arg='normals',
       type='torch.Tensor',
       help='Nx3 list of normals for points for normal filter'},
      {arg='erosion_filter',
       type="boolean",
       help="do we compute erosion", default=false},
      {arg='erosion_function',
       type="function",
       help="closure to compute erosion"},
      {arg="debug_info", type="table", help="debug information to be added to the plane"}
               )
            
            collectgarbage()
            local current_plane = nil
            local error_string  = nil

   -- block for finding planes based on seed or from support of expanded set
   while true do -- this is a dummy block to avoid a string of nested ifs with breaks.
      local seed_npts = mask:sum()
      local mask_size = mask:size()
      -- 1) 1st test: seed has a minimum number of points to form a plane
      if ( seed_npts < 3) then
         error_string="not enough seed points"
         break
      end

      local seed_pts = select_points(points,mask)

      local seed_plane_eqn = fit_plane(seed_pts)
      local seed_score     = score(seed_pts,seed_plane_eqn)

      -- 2) 2nd test: points in the segment explain a plane (no outliers in the segment)
      if (seed_score > threshold) then
         error_string="seed plane above threshold"
         break
      end
      
      local seed_explained_pts, seed_explained_npts, seed_explained_mask =
         find_points_explained_by_plane(points, seed_plane_eqn, threshold, 
                                        normal_filter, normal_threshold, normals)
      
      -- 3) 3rd test: seed plane explains the minimum number of points. eg. could be 0 after an erosion
      
      if seed_explained_npts < min_points_for_seed then
         error_string="seed explained points (after filtering) do not explain enough points for seed"
         break
      end

      local seed_explained_score = score(seed_explained_pts,seed_plane_eqn)

      -- 4) 4th test if seed explains enough points and points are below threshold we have a plane
      if (seed_npts > min_points_for_seed) and (seed_explained_score < threshold) then
         -- the plane from original seed that explains a minumum number of points
         current_plane =
            {eqn         = seed_plane_eqn,
             npts        = seed_explained_npts,
             score       = seed_explained_score,
             mask        = seed_explained_mask:resize(mask_size),
             debug_info  = debug_info
            }
      end

      if not expanded_points then
         if current_plane then 
            error_string="found plane"
         else
            error_string="no plane found. not testing expanded points"
            printf(" - no plane found: seed npts: %d score: %f, %f", seed_npts, seed_score, seed_explained_score)
         end
         break
      end
      -- see if we can find better plane with the larger support of points explained by plane

      -- expand support to all points explained by seed, to test a better plane
      local expanded_plane_eqn  = fit_plane(seed_explained_pts)

      local expanded_set_pts, expanded_set_npts, expanded_set_mask =
         find_points_explained_by_plane(points, expanded_plane_eqn, threshold, 
                                        normal_filter, normal_threshold, normals)

      -- 4) don't have a valid plane from seed or expanded set
      if (expanded_set_npts < min_points_for_plane) and (seed_npts < min_points_for_plane) then
         error_string = "plane fit on expanded set does not explain enough points for plane"
         break
      end

      -- 5) 5rd test plane fit on the expanded set.  Does it do better than plane fit on segment?
      --  5a) is it a good plane and better than what we have
      if (expanded_set_npts < seed_npts) then
         error_string="plane fit on expanded set explains fewer points than plane on seed"
         break
      end

      local expanded_set_score = score(expanded_set_pts,expanded_plane_eqn)

      -- 5b) is the score below the threshold
      if (expanded_set_score > threshold) then
         error_string = "plane fit on expanded set does not fit own points"
         break
      end

      current_plane =
         {eqn        = expanded_plane_eqn,
          npts       = expanded_set_npts,
          score      = expanded_set_score,
          mask       = expanded_set_mask:resize(mask_size),
          debug_info = debug_info
         }
      error_string = "found best plane with expanded set"
      break
   end -- break out of dummy loop for plane finding
   return current_plane, error_string
end

function pf.find_rotation_to_align_axis(norm)
   local angle_az = geom.util.unit_cartesian_to_spherical_angles(planes[3].eqn)
   -- TODO check this again.  Disconnect btw. 2D projections and 3D.  I would have expected azimuth in first
   return geom.quaternion.from_euler_angle(torch.Tensor({0,angle_az[1]}))
end

function pf.axis_align(pts,quat)
   quat = quat or find_rotation_to_align_axis(pts)
   return geom.quaternion.rotate(quat,pts), quat
end

function pf.dump_pts(pts,fname)
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
function pf.align_planes(planes,allpts)
   maxv = -1
   midx = 1
   for i,p in pairs(planes) do
      if (p.eqn[1] > maxv) and (p.npts > allpts:size(1)*0.01) then
         maxv = p.eqn[1] ;
         midx = i ;
      end
   end

   pm = planes[midx]
   quat = find_rotation_to_align_axis(pm.eqn)
   aapts = axis_align(allpts,quat)
   return quat, aapts
end

function pf.dump_pts_to_xyz (planes, aapts, fname)
   for i,p in pairs(planes) do
      ppts = select_points(aapts,p.mask)
      dump_pts(ppts,string.format("%s_%d_pts.xyz",fname,i))
   end
end

function pf.aligned_planes_to_obj (planes, aapts, quat, fname)
   io = require 'io'
   fname = fname or "planes"

   objf = io.open(fname..".obj",'w')

   -- loop through planes. rotate normals.  find bbx in plane. create rectangle.
   for i,p in pairs(planes) do

      ppts = select_points(aapts,p.mask)
      
      pn = geom.quaternion.rotate(quat,p.eqn)
      pcntr = ppts:mean(1):squeeze()
      pn[4] = -pn[{{1,3}}] * pcntr
      pmin = ppts:min(1):squeeze()
      pmax = ppts:max(1):squeeze()
      s = torch.sign(pn)
      m = torch.abs(pn[{{1,3}}])
      _,mi = m:max(1)
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
   s = "f "
   for i = 1,#planes*4 do
      s = string.format("%s %d",s,i)
      if (i%4) == 0 then
         objf:write(s.."\n")
         s = "f "
      end
   end
   objf:close()
end

return pf
