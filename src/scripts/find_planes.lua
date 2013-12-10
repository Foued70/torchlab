require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")
pi = math.pi
pi2 = pi/2
io               = require 'io'
path             = require 'path'
imgraph          = require "../imgraph/init"
saliency         = require "../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Find planes')
cmd:text()
cmd:text('Options')
cmd:option('-src_dir','arcs/temporary-circle-6132/source/po_scan/a/001/')
cmd:option('-out_dir','output/')
cmd:option('-residual_threshold', 60)
cmd:option('-residual_threshold_for_removal', 20)
cmd:option('-normal_threshold', math.pi/3)
cmd:option('-normal_threshold_for_removal', math.pi/4)
cmd:option('-min_pts_for_seed', 81)  -- < 20 x 20 ...
cmd:option('-min_pts_for_plane', 900) -- 30x30 window is minimum
cmd:option('-scale_factor', 1.8)
cmd:option('-n_scale', 5)
cmd:option('-normal_type', 'var')
cmd:option('-dummy',false)
cmd:option('-dummy2',false)
cmd:option('-non_interactive',false)
-- test new itrw options
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

src_dir               = params.src_dir:gsub("/*$","")
wrk_dir               = src_dir:gsub("source","work")

pcfiles               = util.fs.glob(src_dir, {"xyz$"})
base_out_dir          = params.out_dir

-- new scans are in mm which makes most default measurements 1000x off...
max_radius            = 25000

residual_threshold    = tonumber(params.residual_threshold)
normal_threshold      = tonumber(params.normal_threshold)
residual_threshold_for_removal    = tonumber(params.residual_threshold_for_removal)
normal_threshold_for_removal      = tonumber(params.normal_threshold_for_removal)

min_points_for_seed   = tonumber(params.min_pts_for_seed)
min_points_for_plane  = tonumber(params.min_pts_for_plane)

-- saliency
base_win              = math.floor(math.sqrt(min_points_for_seed))
scale_factor          = tonumber(params.scale_factor)
n_scale               = params.n_scale

graph_merge_threshold = tonumber(params.graph_merge_threshold)
-- flags
normal_type        = params.normal_type

validate = Plane.Validate.new{
   residual_threshold   = residual_threshold,
   normal_threshold     = normal_threshold,
   min_points_for_seed  = min_points_for_seed,
   min_points_for_plane = min_points_for_plane
}
itrw = Plane.FitIterativeReweighted.new{
   residual_threshold = residual_threshold,
   residual_decr      = 0.7,
   residual_stop      = 1,
   normal_threshold   = normal_threshold,
   normal_decr        = 0.7,
   normal_stop        = math.pi/360,
   min_points_for_plane = min_points_for_plane
}

itrw.save_images = true
itrw.verbose     = true

matcher = Plane.Matcher.new(residual_threshold, normal_threshold)

for pci,pcfile in pairs(pcfiles) do
   _G.planes    = {}
   error_counts = {}
   local count  = 1

   -- setup outfile naming
   out_dir = base_out_dir .. "/".. pcfile:gsub("[/%.]","_")
   out_dir = string.format("%s/iterative_saliency_base_%d_scale_%1.1f_n_scale_%d",
                           out_dir, base_win, scale_factor, n_scale)
   
   out_dir = string.format("%s_thres_%d_normthres_%2.4f_minseed_%d_minplane_%d",
                           out_dir,
                           residual_threshold, normal_threshold,
                           min_points_for_seed, min_points_for_plane)
   
   out_dir = out_dir .. "_normal_"..normal_type
   print("Making ".. out_dir)
   if not os.execute(string.format("mkdir -p %s", out_dir)) then
      error("error setting up  %s", out_dir)
   end

   file_bname = path.basename(out_dir)
   -- short circuit for restarting batch runs
   if params.non_interactive and util.fs.is_file(out_dir .. "/planes.t7") then
      print("SKIPPING already done")
   else
      log.tic()
      -- load pointcloud
      _G.pc     = PointCloud.PointCloud.new(pcfile, max_radius)
      points    = pc:get_xyz_map()

      -- only var map
      normals,dd,phi,theta,norm_mask = pc:get_normal_map()
      if (normal_type == "smooth") then
         normals,dd,phi,theta,norm_mask = pc:get_smooth_normal()
      end

      collectgarbage()

      -- norm_mask == 1 where normals are bad, our mask == 1 where pts are valid
      local initial_valid    = norm_mask:eq(0):double()
      local accumulate_valid = initial_valid:clone()
      local patch_mask = torch.ByteTensor(accumulate_valid:size()):zero()

      local imgh = normals:size(2)
      local imgw = normals:size(3)

      -- compute all the scales at once and thus reuse the integer image
      local salient = saliency.high_entropy_features{img=normals,
                                                     kr=base_win,kc=base_win,
                                                     scalefactor=scale_factor,
                                                     nscale=n_scale}


      -- TODO different x and y window sizes
      -- find patches for initial candidate_planes
      -- avoid huge zones of zero salience in clean scans by convolving with gaussian.
      -- log accentuates the differences near zero. reduces difference far from zero.

      local final_win   = math.floor(base_win * scale_factor^(n_scale-1))
      local inv_salient = image.convolve(salient:reshape(1,salient:size(1), salient:size(2)),
                                         image.gaussian(base_win),'same'):contiguous():squeeze()
      inv_salient:add(1):log()

      local max_saliency = inv_salient:max()
      inv_salient:add(-max_saliency):abs()

      local half_win = math.floor(final_win * 0.5)
      printf("final win: %f half_win: %f max saliency: %f", final_win, half_win, max_saliency)

      -- reapply border      
      inv_salient[{{1,half_win},{}}]         = 0
      inv_salient[{{},{1,half_win}}]         = 0
      inv_salient[{{imgh-half_win,imgh},{}}] = 0
      inv_salient[{{},{imgw-half_win,imgw}}] = 0

      local distance_weights = accumulate_valid:clone()
      -- do non-maximal suppression on smallest window size (faster) and don't miss small planes.
      local nms     = image.NonMaximalSuppression.new(base_win,base_win)
      local orig_mx = nms:forward(inv_salient):squeeze()
      local mx      = torch.cmul(orig_mx,distance_weights) -- make sure we don't count masked areas as non-salient

      n_patches = mx:gt(1):sum()

      while (n_patches > 1) do
         printf(" - %d patches left with window %d,%d",n_patches, base_win, base_win)
         -- find a single max value to process
         idx,val = Plane.finder_utils.get_max_val_index(mx)
         -- remove this value whether accepted or not
         bbx = Plane.finder_utils.compute_bbx(idx,base_win,base_win,imgh,imgw)

         patch_mask:fill(0)
         patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1
         -- patch_mask:cmul(accumulate_valid)

         -- TODO change name of finder to validate
         current_plane, error_string =
            validate:seed(points, normals, patch_mask, debug_info)
         
         local plane_center = nil
         
         if current_plane then
            current_plane:residual_threshold(residual_threshold_for_removal)
            current_plane:normal_threshold(normal_threshold_for_removal)

            current_plane.debug_info = {{
                                           win_idx  = idx[{{},i}]:clone(),
                                           win_size = {x=base_win, y=base_win}
                                        }}

            itrw.image_id = string.format("%s/plane_%04d_",out_dir,#planes+1)

            plane_center = current_plane.center

            best_plane, best_score, best_n_points, curves, new_weights =
               itrw:fit(points, normals:reshape(3,normals:nElement()/3),
                        current_plane.eqn, plane_center,
                        distance_weights)

            current_plane.eqn = best_plane

            -- check filtering (not removing enough points)
            local explained_pts, explained_n_pts, explained_mask =
               current_plane:filter_points(points, normals)

            -- TODO validate refit plane
            current_plane.n_pts  = explained_n_pts
            current_plane.score  = best_score
            current_plane.mask   = explained_mask:resize(patch_mask:size())
            current_plane.curves = curves

            table.insert(planes, current_plane)
            score = matcher:match(planes, points, normals)

            distance_weights = score:mul(1/score:max()):add(-1):abs():cmul(initial_valid)

            image.save(string.format("%spatch_mask.jpg", itrw.image_id),image.combine(patch_mask))
            gnuplot.pngfigure(string.format("%splot.png",itrw.image_id))
            gnuplot.xlabel("residual threshold in mm")
            gnuplot.ylabel("number of points within scoring thresholds")
            gnuplot.raw("set key bottom right")
            gnuplot.plot(curves)
            gnuplot.close()

            image.save(string.format("%smask.jpg",itrw.image_id), image.combine(current_plane.mask))

            image.save(string.format("%svalid.jpg",itrw.image_id), image.combine(accumulate_valid))

            printf(" - result: %s", error_string)

            if error_counts[error_string] then
               error_counts[error_string] = error_counts[error_string] + 1
            else
               error_counts[error_string] = 1
            end
            if (#planes > 0) and (#planes % 5 == 0) then
               local imgname = string.format("%s/scale_%dx%d.jpg",out_dir,base_win,base_win)
               print("saving "..imgname)
               image.save(imgname, Plane.finder_utils.visualize_planes(planes))
            end
         end

         printf("+ Total %d planes in %d patches with %d left", #planes, count, n_patches)
         printf("   - max mx: %f min mx: %f", mx:max(), mx:min())
         count = count + 1

         -- clear seed from valid points so we don't check the same patch twice
         accumulate_valid[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 0
         mx = torch.cmul(orig_mx,distance_weights):cmul(accumulate_valid)
         -- DEBUG
         image.save(string.format("%s/mx_%03d.jpg",out_dir,count),image.combine(mx))
         image.save(string.format("%s/valid_%03d.jpg",out_dir,count),image.combine(accumulate_valid))
         image.save(string.format("%s/distance_%03d.jpg",out_dir,count),
                    image.combine(distance_weights:reshape(imgh,imgw)))
         printf("mx max: %f min: %f", mx:max(), mx:min())
         --  DEBUG down weighted saliency threshold
         -- printf(" - gt(0.1) : %d", mx:gt(0.1):sum())
         -- printf(" - gt(0.5) : %d", mx:gt(0.5):sum())
         -- printf(" - gt(1) : %d", mx:gt(1):sum())
         -- printf(" - gt(2) : %d", mx:gt(2):sum())
         -- printf(" - gt(3) : %d", mx:gt(3):sum())
         -- printf(" - gt(4) : %d", mx:gt(4):sum())
         -- ]]
         n_patches = mx:gt(1):sum()

         collectgarbage()

      end -- while still valid in this scale
      if #planes > 0 then     
         local planefilename = string.format("%s/planes-tmp.t7",out_dir)
         print("saving "..planefilename)
         torch.save(planefilename, planes)
         local imgname = string.format("%s/scale_%dx%d.jpg",out_dir,base_win,base_win)
         print("saving "..imgname)
         image.save(imgname, Plane.finder_utils.visualize_planes(planes))
      end
   end -- check SKIP
end -- loop through all plane files

-- done
local planefilename_tmp = string.format("%s/planes-tmp.t7",out_dir)
local planefilename = string.format("%s/planes.t7",out_dir)
os.execute("mv ".. planefilename_tmp .. " " .. planefilename)

-- quit out of luvit if
if params.non_interactive then
   process.exit()
end
