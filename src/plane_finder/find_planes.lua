pi = math.pi
pi2 = pi/2
io               = require 'io'
path             = require 'path'
imgraph          = require "../imgraph/init"
saliency         = require "../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual
plane_finder     = require './plane_finder.lua'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-src_dir','arcs/temporary-circle-6132/source/po_scan/a/001/')
cmd:option('-out_dir','output/')
cmd:option('-thres', 10)
cmd:option('-min_pts_for_seed', 150)  -- < 20 x 20 ...
cmd:option('-min_pts_for_plane', 900) -- 30x30 window is minimum
cmd:option('-graph_merge_thres', 5)
cmd:option('-scale_factor', 1.2)
cmd:option('-n_scale', 5)
cmd:option('-normal_filter', true)
cmd:option('-expanded_points', false)
cmd:option('-use_saliency', false)
cmd:option('-normal_type', 'raw')
cmd:option('-dummy',false)
cmd:option('-dummy2',false)
cmd:option('-non_interactive',false)
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

src_dir = params.src_dir:gsub("/*$","")
wrk_dir = src_dir:gsub("source","work")
pcfiles = util.fs.glob(src_dir, {"xyz$"})
base_out_dir = params.out_dir

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000

threshold             = tonumber(params.thres)
normal_threshold      = math.cos(math.pi/6)
graph_merge_threshold = tonumber(params.graph_merge_thres)


-- TODO add these to args
min_points_for_seed   = tonumber(params.min_pts_for_seed)
min_points_for_plane  = tonumber(params.min_pts_for_plane)

-- saliency
base_win      = math.floor(math.sqrt(min_points_for_seed))
scale_factor  = tonumber(params.scale_factor)
n_scale       = params.n_scale

-- flags
normal_filter     = params.normal_filter
erosion_filter    = false
expanded_points   = params.expanded_points
add_planes_bool   = params.add_planes
combine_planes_every  = tonumber(params.combine_planes_every)
use_saliency      = params.use_saliency
normal_type       = params.normal_type

for _,pcfile in pairs(pcfiles) do
   local planes = {}
   _G.test_planes = planes
   _G.error_counts = {}
   _G.add_scores = {}
   _G.cmb_scores = {}
   _G.count    = 1
   last_tested = 1

   -- setup outfile naming
   out_dir = base_out_dir .. "/".. pcfile:gsub("[/%.]","_")
   if use_saliency then
      out_dir = string.format("%s/saliency_base_%d_scale_%1.1f_n_scale_%d",
                              out_dir, base_win, scale_factor, n_scale)
   else
      out_dir = string.format("%s/segmentation_merge_%1.0f",
                              out_dir, graph_merge_threshold)
   end

   out_dir = string.format("%s_thres_%d_minseed_%d_minplane_%d",
                           out_dir,
                           threshold, min_points_for_seed, min_points_for_plane)

   if normal_filter   then out_dir = out_dir .. string.format("_nf_%1.2f", normal_threshold)  end
   if expanded_points then out_dir = out_dir .. "_exp" end
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
      xyz_map   = pc:get_xyz_map()
      allpts    = xyz_map:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()
      _G.test_points = allpts

      normals,dd,phi,theta,norm_mask = pc:get_normal_map()
      if (normal_type == "var") then
         normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
      elseif (normal_type == "var_smooth") then
         normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
         normals,phi,theta,dd,norm_mask = pc:get_smooth_normal(nil,nil,nil,phi,theta,norm_mask)
      elseif (normal_type == "smooth") then
         normals,dd,phi,theta,norm_mask = pc:get_smooth_normal()
      end

      local allnrm = torch.Tensor(3,xyz_map:size(2)*xyz_map:size(3))
      allnrm[{{1,3},{}}]:copy(normals)
      -- allnrm[{4,{}}]:copy(dd)
      allnrm = allnrm:t():contiguous()
      _G.test_nrm = allnrm
      collectgarbage()

      -- norm_mask == 1 where normals are bad, our mask is where pts are valid
      _G.initial_valid = norm_mask:eq(0)
      _G.valid = initial_valid:clone()

      local imgh = normals:size(2)
      local imgw = normals:size(3)

      if use_saliency then

         -- compute all the scales at once and thus reuse the integer image
         _G.salient,_G.sc = saliency.high_entropy_features{img=normals,
                                                           kr=base_win,kc=base_win,
                                                           scalefactor=scale_factor,
                                                           nscale=n_scale}

         patch_mask = valid:clone():zero()

         for scale = n_scale,1,-1 do

            -- TODO different x and y window sizes
            local final_win = base_win * scale_factor ^ (scale -1)
            -- find patches for initial candidate_planes
            local inv_salient  = sc[scale]:clone()

            local max_saliency = inv_salient:max()
            inv_salient:add(-max_saliency):abs()

            local nms    = image.NonMaximalSuppression.new(final_win,final_win)
            local mx     = nms:forward(inv_salient):squeeze()
            mx:cmul(valid:double()) -- make sure we don't count masked areas as non-salient

            -- randomize the output as there are such flat non-disriminate areas
            local bmx = mx:gt(0)
            mx:add(torch.rand(mx:size()):mul(1/mx:max())):cmul(bmx:double())

            n_patches = bmx:sum()

            while (n_patches > 1) do

               printf(" - %d patches left with window %d,%d",n_patches, final_win, final_win)
               -- speed this up for single batch
               idx,val = plane_finder.get_max_val_index(mx)
               -- printf("max saliency = %f %f %f", mx:max(), mx[{idx[1],idx[2]}], val)

               -- remove this value whether accepted or not
               bbx = plane_finder.compute_bbx(idx,final_win,final_win,imgh,imgw)

               patch_mask:fill(0)
               patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1
               patch_mask:cmul(valid)

               current_plane, error_string =
                  plane_finder.from_seed{
                     points               = allpts,
                     mask                 = patch_mask,
                     threshold            = threshold,
                     min_points_for_seed  = min_points_for_seed,
                     min_points_for_plane = min_points_for_plane,
                     expanded_set         = explanded_set,
                     normal_filter        = normal_filter,
                     normals              = allnrm,
                     normal_threshold     = normal_threshold,
                     erosion_filter       = erosion_filter,
                     erosion_function     = erodeDilate,
                     debug_info           = {{
                                                win_idx  = idx[{{},i}]:clone(),
                                                win_size = {x=final_win, y=final_win}
                                             }}
                  }

               -- clear mx so we don't check the same patch twice

               mx[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 0
               n_patches = mx:gt(0):sum()

               if current_plane then
                  -- don't keep checking windows already explained by a plane
                  mx[current_plane.mask] = 0
                  plane_finder.recompute_valid_points(valid,planes)

                  table.insert(planes, current_plane)
               end

               printf(" - result: %s", error_string)

               if error_counts[error_string] then
                  error_counts[error_string] = error_counts[error_string] + 1
               else
                  error_counts[error_string] = 1
               end


               printf(" - Total %d planes in %d patches with %d left", #planes, count, n_patches)
               count = count + 1
               collectgarbage()

            end -- while still valid in this scale
            if (#planes > 0) then
               image.save(string.format("%s/scale_%dx%d.png",out_dir,final_win,final_win),
                          plane_finder.visualize_planes(planes))
            end
         end -- for scales
      else

         -- make segm patches
         _G.data = torch.Tensor(3,phi:size(1),phi:size(2))

         data[1]:copy(phi):add(-phi:min()):mul(1/(phi:max()-phi:min()))
         data[2]:copy(theta):add(-theta:min()):mul(1/(theta:max()-theta:min()))
         d = data[3]
         d:copy(torch.cmul(xyz_map,normals):resize(3,xyz_map[1]:nElement()):sum(1):squeeze())
         d:add(1):log()
         d:mul(1/d:max())

         datag = image.convolve(data, image.gaussian(graph_merge_threshold), 'same')
         graph = imgraph.graph(datag)
         mstsegm = imgraph.segmentmst(graph, graph_merge_threshold, min_points_for_seed)

         graph_rgb = image.combine(imgraph.colorize(mstsegm)):clone()

         h = {}
         n_segm = 0
         mstsegm:apply(function (x)
                          if h[x] then
                             h[x] = h[x] + 1
                          else
                             n_segm = n_segm + 1
                             h[x] = 1
                          end
                       end)

         printf("Found %d segments",n_segm)
         _G.v = torch.LongTensor(2,n_segm)
         patch_count = 1
         for id,quant in pairs(h) do
            v[1][patch_count] = quant
            v[2][patch_count] = id
            patch_count = patch_count+1
         end

         s, si = torch.sort(v[1])

         for ci = n_segm,1,-1 do
            ii = si[ci]
            segm_count = v[1][ii]
            segm_id    = v[2][ii]

            printf(" - processing segm id: %d with %d pixels", segm_id, segm_count)

            segm_mask = mstsegm:eq(segm_id):cmul(valid)

            current_plane, error_string =
               plane_finder.from_seed{
                  points               = allpts,
                  mask                 = segm_mask,
                  threshold            = threshold,
                  min_points_for_seed  = min_points_for_seed,
                  min_points_for_plane = min_points_for_plane,
                  expanded_set         = explanded_set,
                  normal_filter        = normal_filter,
                  normals              = allnrm,
                  normal_threshold     = normal_threshold,
                  erosion_filter       = erosion_filter,
                  erosion_function     = erodeDilate,
                  debug_info           = {{
                                             segm_ids = segm_id,
                                             segm_counts = segm_count
                                          }}
               }

            collectgarbage()

            printf(" - error: %s", error_string)

            if error_counts[error_string] then
               error_counts[error_string] = error_counts[error_string] + 1
            else
               error_counts[error_string] = 1
            end
            if current_plane then
               table.insert(planes, current_plane)
            end
            printf(" - Total %d planes so far in %d/%d", #planes, count, n_segm)
            count = count + 1

         end

         collectgarbage()

      end

      toc = log.toc()

      -- score
      n_planes = #planes
      pstd = torch.Tensor(n_planes)

      for i,p in pairs(planes) do
         valid[p.mask] = 0
         pstd[i]       = p.score
      end
      remain_pts     = valid:sum()
      total_pts      = initial_valid:sum()
      percent_remain = 100 * remain_pts/total_pts
      found_pts      = total_pts - remain_pts
      percent_found  = 100 - percent_remain

      if (n_planes > 0) then
         rgb = plane_finder.visualize_planes(planes)
      end

      _G.outname = out_dir .. "/planes"
      image.save(outname .. ".png", rgb)
      if not use_saliency then
         image.save(out_dir  .. "/segmentation.png", graph_rgb)
      end
      image.save(out_dir  .. "/normals.png", image.combine(normals))
      image.save(out_dir  .. "/valid.png", valid:mul(255))

      -- write score
      score_fname = outname .. "-score.txt"
      score_file = io.open(score_fname,"w")
      score_file:write("Errors:\n")
      for str,cnt in pairs(error_counts) do
         score_file:write(string.format("%s %d\n",str,cnt))
      end
      score_file:write("Scores:\n")
      score_file:write(string.format("patches searched %d\n",count))
      score_file:write(string.format("planes found %d\n",#planes))
      score_file:write(string.format("score mean %f min %f max %f \n",pstd:mean(),pstd:min(), pstd:max()))
      score_file:write(string.format("points explained %d/%d %2.1f%%\n",found_pts, total_pts, percent_found))
      score_file:write(string.format("time: %fs\n", toc*1e-3))
      score_file:write("Add Scores:\n")
      for str,tbl in pairs(add_scores) do
         score_file:write(string.format("%s %d\n", str, tbl.cnt))
      end
      for str,tbl in pairs(cmb_scores) do
         score_file:write(string.format("%s %d\n", str, tbl.cnt))
      end

      score_file:close()
      os.execute("cat "..score_fname)

      -- write scoreline
      score_fname = outname .. "-scoreline.txt"
      score_file = io.open(score_fname,"w")
      score_file:write("# saliency base_win scale_factor n_scale ")
      score_file:write("segmentation graphmerge ")
      score_file:write("norm_1=raw,2=smooth,3=var,4=var_smooth normal_threshold ")
      score_file:write("minseed minplane ")
      score_file:write("psearched pfound scmean scmin scmax found_pts remain_pts total_pts percent_found time\n")
      score_file:write(string.format("%s ", file_bname))
      score_file:write(string.format("%d %d %f %d ", use_saliency and 1 or 0, base_win, scale_factor, n_scale))
      score_file:write(string.format("%d %d ", use_saliency and 0 or 1, graph_merge_threshold))
      score_file:write(string.format("%d %f ",
                                     normal_type == "raw" and 1 or
                                        (normal_type == "smooth" and 2) or
                                        (normal_type == "var" and 3) or
                                        (normal_type == "var_smooth" and 4) or 0,
                                     normal_threshold))
      score_file:write(string.format("%d %d ", min_points_for_seed, min_points_for_plane))
      score_file:write(string.format("%d ",count))
      score_file:write(string.format("%d ",#planes))
      score_file:write(string.format("%f %f %f ",pstd:mean(),pstd:min(), pstd:max()))
      score_file:write(string.format("%d %d %d %2.1f ",found_pts, remain_pts, total_pts, percent_found))
      score_file:write(string.format("%f\n", toc*1e-3))
      score_file:close()

      os.execute("cat "..score_fname)

      collectgarbage()
      -- save obj
      quat, aapts = plane_finder.align_planes(planes,allpts)
      plane_finder.aligned_planes_to_obj (planes, aapts, quat, outname..".obj")

      -- save planes with out masks
      for _,p in pairs(planes) do
         p.mask = nil
      end
      collectgarbage()
      torch.save(outname .. ".t7", planes)
   end
end -- loop through all plane files
-- quit out of luvit if
if params.non_interactive then
   process.exit()
end
