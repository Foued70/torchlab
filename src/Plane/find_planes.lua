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
cmd:text('Compute image projections')
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
cmd:option('-graph_merge_threshold', 5)
cmd:option('-scale_factor', 1.8)
cmd:option('-n_scale', 5)
cmd:option('-expanded_set', false)
cmd:option('-iterative', false)
cmd:option('-pin_center', false)
cmd:option('-use_saliency', false)
cmd:option('-normal_type', 'var')
cmd:option('-dummy',false)
cmd:option('-dummy2',false)
cmd:option('-non_interactive',false)
-- test new itrw options
cmd:option('-down_weight_iterative',false)
cmd:option('-use_slope_score',false)
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
iterative_reweight = params.iterative
pin_center         = params.pin_center
use_saliency       = params.use_saliency
normal_type        = params.normal_type

down_weight_iterative = params.down_weight_iterative
use_slope_score       = params.use_slope_score

finder = Plane.Finder.new{
   residual_threshold   = residual_threshold,
   normal_threshold     = normal_threshold,
   min_points_for_seed  = min_points_for_seed,
   min_points_for_plane = min_points_for_plane
}
finder.use_slope_score = use_slope_score

itrw = Plane.FitIterativeReweighted.new{
   residual_threshold = residual_threshold,
   residual_decr      = 0.7,
   residual_stop      = 1,
   normal_threshold   = normal_threshold,
   normal_decr        = 0.7,
   normal_stop        = math.pi/360,
   min_points_for_plane = min_points_for_plane
}

itrw.save_images = false
itrw.verbose     = false
itrw.use_slope_score = params.use_slope_score

for pci,pcfile in pairs(pcfiles) do
   _G.planes    = {}
   error_counts = {}
   local count     = 1

   -- setup outfile naming
   out_dir = base_out_dir .. "/".. pcfile:gsub("[/%.]","_")
   if use_saliency then
      out_dir = string.format("%s/saliency_base_%d_scale_%1.1f_n_scale_%d",
                              out_dir, base_win, scale_factor, n_scale)
   else
      out_dir = string.format("%s/segmentation_merge_%1.0f",
                              out_dir, graph_merge_threshold)
   end

   out_dir = string.format("%s_thres_%d_normthres_%2.4f_minseed_%d_minplane_%d",
                           out_dir,
                           residual_threshold, normal_threshold,
                           min_points_for_seed, min_points_for_plane)

   if normal_filter   then out_dir = out_dir .. string.format("_nf_%1.2f", normal_threshold)  end
   if expanded_set then out_dir = out_dir .. "_expanded" end
   if iterative_reweight then out_dir = out_dir .. "_iterative" end
   if use_slope_score then out_dir = out_dir .. "_slope_score" end
   if down_weight_iterative then out_dir = out_dir .. "_down_weight" end
   if pin_center then out_dir = out_dir .. "_pinned_center" end
   if search_multi_scale_saliency then out_dir = out_dir .. "_multi_sal" end
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

      -- norm_mask == 1 where normals are bad, our mask is where pts are valid
      initial_valid = norm_mask:eq(0)
      valid = initial_valid:clone()
      local patch_mask = torch.ByteTensor(valid:size()):zero()

      local imgh = normals:size(2)
      local imgw = normals:size(3)

      -- compute all the scales at once and thus reuse the integer image
      local salient = saliency.high_entropy_features{img=normals,
                                                     kr=base_win,kc=base_win,
                                                     scalefactor=scale_factor,
                                                     nscale=n_scale}


      -- TODO different x and y window sizes
      -- find patches for initial candidate_planes
      -- avoid huge zones of zero salience in clean scans by adding a little noise.
      -- log accentuated the differences near zero. reduces difference far from zero.
      local   inv_salient = salient:clone():add(1):add(torch.rand(salient:size()):mul(0.001)):log()

      local max_saliency = inv_salient:max()
      inv_salient:add(-max_saliency):abs()

      local cumulative_weights = valid:double()
      -- do non-maximal suppression on smallest window size (faster) and don't miss small planes.
      local nms     = image.NonMaximalSuppression.new(base_win,base_win)
      local mx      = nms:forward(inv_salient):squeeze()
      mx:cmul(cumulative_weights) -- make sure we don't count masked areas as non-salient
      -- randomize the output as there are such flat non-disriminate areas
      local bmx = mx:gt(0)
      if not down_weight_iterative then
         cumulative_weights = nil
      end
      n_patches = bmx:sum()
      while (n_patches > 1) do
         printf(" - %d patches left with window %d,%d",n_patches, base_win, base_win)
         -- find a single max value to process
         idx,val = Plane.finder_utils.get_max_val_index(mx)
         -- remove this value whether accepted or not
         bbx = Plane.finder_utils.compute_bbx(idx,base_win,base_win,imgh,imgw)

         patch_mask:fill(0)
         patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1
         patch_mask:cmul(valid)

         -- TODO change finder to validate
         current_plane, error_string =
            finder:validate_seed(points, normals, patch_mask, debug_info)

         -- clear mx so we don't check the same patch twice
         mx[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 0
         n_patches = mx:gt(0):sum()

         local plane_center = nil

         if current_plane then
            current_plane:residual_threshold(residual_threshold_for_removal)
            current_plane:normal_threshold(normal_threshold_for_removal)

            current_plane.debug_info = {{
                                           win_idx  = idx[{{},i}]:clone(),
                                           win_size = {x=base_win, y=base_win}
                                        }}

            itrw.image_id = string.format("%s/plane_%04d_",out_dir,#planes+1)

            if pin_center then
               plane_center = current_plane.center
            end

            best_plane, best_score, best_n_points, curves, new_weights =
               itrw:fit(points, normals:reshape(3,normals:nElement()/3),
                        current_plane.eqn, plane_center,
                        cumulative_weights)

            if cumulative_weights then
               cumulative_weights:cmul(new_weights:add(-new_weights:max()):abs())
            end

            current_plane.eqn = best_plane

            -- check filtering (not removing enough points)
            local explained_pts, explained_n_pts, explained_mask =
               current_plane:filter_points(points, normals)

            -- TODO validate refit plane
            current_plane.n_pts  = explained_n_pts
            current_plane.score  = best_score
            current_plane.mask   = explained_mask:resize(patch_mask:size())
            current_plane.curves = curves

            image.save(string.format("%spatch_mask.jpg", itrw.image_id),image.combine(patch_mask))
            gnuplot.pngfigure(string.format("%splot.png",itrw.image_id))
            gnuplot.xlabel("residual threshold in mm")
            gnuplot.ylabel("number of points withing scoring thresholds")
            gnuplot.raw("set key bottom right")
            gnuplot.plot(curves)
            gnuplot.close()

            image.save(string.format("%smask.jpg",itrw.image_id), image.combine(current_plane.mask))
            -- don't keep checking windows already explained by a plane
            mx[current_plane.mask] = 0
            valid:cmul(current_plane.mask:eq(0))
            image.save(string.format("%svalid.jpg",itrw.image_id), image.combine(valid))
            table.insert(planes, current_plane)

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
         collectgarbage()

      end -- while still valid in this scale
      local imgname = string.format("%s/scale_%dx%d.jpg",out_dir,base_win,base_win)
      print("saving "..imgname)
      image.save(imgname, Plane.finder_utils.visualize_planes(planes))
   end -- check SKIP
end -- loop through all plane files

-- quit out of luvit if
if params.non_interactive then
   process.exit()
end
