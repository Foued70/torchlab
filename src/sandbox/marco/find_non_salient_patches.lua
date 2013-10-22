pi = math.pi
pi2 = pi/2
io               = require 'io'
saliency         = require "../../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual

plane_finder = require './plane_finder.lua'

src_dir = 'arcs/temporary-circle-6132/source/po_scan/a/001/'
wrk_dir = src_dir:gsub("source","work")

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000

pcfile = src_dir .. 'sweep.xyz'
rawfile = src_dir .. 'sweep.raw'

_G.pc   = PointCloud.PointCloud.new(pcfile, max_radius)
_G.normals,dd,norm_mask = pc:get_normal_map()

-- norm_mask == 1 where normals are bad, our mask is where pts are valid
_G.initial_valid = norm_mask:eq(0)
_G.valid = initial_valid

imgh = normals:size(2)
imgw = normals:size(3)

threshold   = 10 -- in mm
base_win    = 13
scalefactor = 1.4
nscale      = 7
batch_size  = 1

-- compute all the scales at once and thus reuse the integer image
_G.salient,_G.sc = saliency.high_entropy_features{img=normals,
                                                  kr=base_win,kc=base_win,
                                                  scalefactor=scalefactor,
                                                  nscale=nscale}

_G.imgs  = {}
_G.planes = {}

xyz_map = pc:get_xyz_map()
_G.allpts  = xyz_map:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()

-- new erosion operator
erosion_amount = 7
dilate_amount  = 0
smooth_amount  = nil
erodeDilate = nil
if dilate_amount > 0 then 
   erodeDilate = plane_finder.newErodeDilate(erosion_amount,dilate_amount,smooth_amount,imgh,imgw)
else
   erodeDilate = plane_finder.newErode(erosion_amount,imgh,imgw)
end

for scale = nscale,1,-1 do

   -- find patches for initial candidate_planes
   -- whack areas which are masked these should not have low salience.
   _G.inv_salient = sc[scale]:clone()
   max_saliency = inv_salient:max()
   inv_salient:add(-max_saliency):abs()


   -- TODO different x and y window sizes
   final_win = base_win * scalefactor ^ (scale -1)
   _G.nms    = image.NonMaximalSuppression.new(final_win,final_win)
   _G.mx     = nms:forward(inv_salient):squeeze()
   mx:cmul(valid:double()) -- make sure we don't count masked areas as non-salient

   -- TODO fix this bug around the far edge
   bmx_useable_h = imgh-final_win
   bmx_useable_w = imgw-final_win
   n_patches = math.huge
   while (mx:max() > 0) and (n_patches > 0) do
      
      _G.candidate_planes = {}
      printf("max saliency = %f", mx:max())
      _G.bmx    = mx:gt(0)

      idx,val,n_patches = plane_finder.get_vals_index(bmx[{{1,bmx_useable_h},{1,bmx_useable_w}}])

      printf("found %d patches",n_patches)
      if (n_patches > 0) then
         s,si = torch.sort(val)

         tot_win_pts = final_win * final_win
         -- loop through patches validating candidate_planes
         -- when n_patches > batch_size only search in top 1/2 patches 
         local n_patch   = math.min(n_patches-1, 
                                    math.max(batch_size,math.floor(n_patches*0.5-1)))
         local step_size = math.max(1,math.floor(n_patch/batch_size)) -- max batch_size patches searched
         printf("processing batch of %d",(n_patch/step_size))
         for ii = n_patches,n_patches-n_patch,-step_size do
            i = si[ii]
            -- remove this value whether accepted or not
            -- printf("settting mx %f to 0", mx[{idx[{1,i}],idx[{2,i}]}]) 
            bbx = plane_finder.compute_bbx(idx[{{},i}],final_win,final_win,imgh,imgw)

            mx[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 0
            win_pts, n_valid = plane_finder.get_patch_pts(idx[{{},i}],final_win,final_win,xyz_map,valid)
            if (n_valid > (tot_win_pts / scalefactor)) then
               -- printf("[%d] %d of %d valid points in the mask", ii, n_valid, tot_win_pts)
               -- validate plane with points within the saliency window
               local plane_eq  = fit_plane(win_pts)
               local local_score = plane_finder.score(win_pts,plane_eq)
               if local_score < threshold then
                  -- plane validated
                  -- find points explained by plane
                  -- printf("[%d] saliency: %f residual score: %f",ii,val[i],local_score)
                  -- printf("  - plane: %f %f %f %f", plane_eq[1], plane_eq[2], plane_eq[3], plane_eq[4])
                  expanded_set_pts, npts, plane_pts_mask = 
                     plane_finder.find_points(allpts,plane_eq, threshold, erodeDilate)
                  -- expand support to all points within threshold
                  new_plane_eq       = fit_plane(expanded_set_pts)
                  expanded_set_score = plane_finder.score(expanded_set_pts,new_plane_eq)

                  -- count points explained by plane computed with expanded set
                  new_expanded_set_pts, new_npts, new_plane_pts_mask = 
                     plane_finder.find_points(allpts,new_plane_eq, threshold, erodeDilate)

                  -- TODO shrink

                  -- plane_finder.score of all points explained by new plane
                  new_score = plane_finder.score(new_expanded_set_pts, new_plane_eq)

                  -- printf("   - found %d points with new plane vs. %d with old", new_npts, npts)
                  -- printf("   - score of selected points %f vs. score in win %f",new_score, expanded_set__score)

                  if ((new_score < threshold) and (new_npts > npts)) then
                     -- print("  - keeping plane computed from larger support")
                     table.insert(candidate_planes,
                                  {eq       = new_plane_eq,
                                   npts     = new_npts,
                                   score    = new_score,
                                   mask     = new_plane_pts_mask:resize(imgh,imgw),
                                   win_idx  = {idx[{{},i}]:clone()},
                                   win_size = {{x=final_win, y=final_win}}
                                  })
                  else
                     -- print("  - keeping original plane")
                     table.insert(candidate_planes,
                                  {eq       = plane_eq,
                                   npts     = npts,
                                   score    = expanded_set_score,
                                   mask     = plane_pts_mask:resize(imgh,imgw),
                                   win_idx  = {idx[{{},i}]:clone()},
                                   win_size = {{x=final_win, y=final_win}}
                                  })
                  end
               end
            end -- if valid
         end

         combined_planes = nil
         -- loop through all the planes and try to combine them
         if #candidate_planes > 0 then
            if (#candidate_planes > 1) then 
               combined_planes = plane_finder.combine_planes(allpts, candidate_planes,threshold, erodeDilate)
               printf(" - Found %d candidates and %d after combined", #candidate_planes, #combined_planes)
            else
               combined_planes = candidate_planes
            end
            if (not planes) or (#planes == 0) then
               _G.planes = combined_planes
            else
               _G.planes = plane_finder.add_planes(allpts, planes, combined_planes, threshold, erodeDilate)
            end
            printf(" - Total %d planes so far (winsize: %d)", #planes, final_win)

            -- combine post adding
            n_planes = #planes
            if (n_planes > 1) then 
               _G.planes = plane_finder.combine_planes(allpts, planes, threshold, erodeDilate)
               changed = n_planes - #planes
               if (changed > 0) then
                  printf(" - Combined %d pairs of %d leaving %d", changed, n_planes, #planes)
                  _G.valid = plane_finder.recompute_valid_points(initial_valid, planes)
               end
            end
            -- recompute mask
            _G.valid = plane_finder.recompute_valid_points(initial_valid, planes)
         end
         mx:cmul(valid:double()) -- make sure we don't count masked areas as non-salient
      end -- if c>0
      collectgarbage()
   end -- while mx > 0

   image.save(string.format("scale_%dx%d.png",final_win,final_win),plane_finder.visualize_planes(planes))

end
