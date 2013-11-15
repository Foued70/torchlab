require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("wxt")
io = require 'io'
-- plane_finder = require './plane_finder'
src_dir = "arcs/motor-unicorn-0776/source/faro/"
-- s = src_dir .. "sweep_001.xyz"
s = "arcs/temporary-circle-6132/source/po_scan/a/001/sweep.xyz"
if not pc then 
   _G.pc = PointCloud.PointCloud.new(s)
end

_G.pl = torch.load("output/arcs_temporary-circle-6132_source_po_scan_a_001_sweep_xyz/combined__thres_40_minplane_150_nf_0.87_normal_var_baseplanes_saliency_base_12_scale_1.2_n_scale_5_thres_20_minseed_150_minplane_900_nf_0.87_normal_var/planes.t7")

-- _G.pls = plane_finder.Planes.new(pl)

_G.xyz_map    = pc:get_xyz_map()
_G.map_height = xyz_map:size(2)
_G.map_width  = xyz_map:size(3)
_G.points     = xyz_map:reshape(xyz_map:size(1),map_height*map_width)

_G.normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
normals:resize(3,map_height*map_width)

_G.ss = nn.SoftShrink(0.1)

score_res_thres  = 100
score_norm_thres = math.pi/3

for pi = 1,#pl do 
   p = pl[pi]
   res_scale = score_res_thres
   nrm_scale = score_norm_thres

   -- class iterated reweighted ?   
   local residuals   = geom.linear_model.residual_fast(points,p.eqn)
   local cosine_dist = plane_finder.util.cosine_distance(normals:t(),p.eqn)
   -- TODO score 2D 
   -- plane_finder.util.score_plane(p.eqn, points, score_res_thres)
   --       plane_finder.util.score_plane_with_normal_threshold(
   local orig_s, orig_x, orig_c, orig_n_pts = 
      plane_finder.util.score_plane_2D(
         p.eqn, points, normals, score_res_thres,  score_norm_thres, n_measurements)
   local curves = {{string.format("iteration: %d thres: %2.1f mm norm: %0.3f rad",
                                  0 , res_scale,nrm_scale),orig_c }}
   local prev_s     = orig_s
   local prev_n_pts = orig_n_pts
   local doresidual = false

   -- TODO other convergence
   for li = 1,128 do 
      collectgarbage()
      -- TODO don't need to recompute all this ... put in get_nbhd function.
      local x = residuals:clone()
      x:div(res_scale)
      x:resize(map_height,map_width)
      local residual_out = util.stats.gaussian_kernel(x)
      
      local n = cosine_dist:clone()
      n:div(nrm_scale)
      n:resize(map_height,map_width)
      local normal_out = util.stats.gaussian_kernel(n)
      
      -- TODO some mixing ? alpha = 0.5
      combo = residual_out:clone()
      combo:cmul(normal_out)
      local combo_out = ss:forward(combo):clone()
      
      -- batch with neighborhood distance
      mask = combo_out:gt(0)
      npts = mask:sum()
      if npts < 100 then
         printf("only %d points in neighborhood. Stopping",npts)
         break
      else
         filtered_points = torch.Tensor(3,npts)
         for i = 1,3 do 
            filtered_points[i] = points[i][mask] 
         end
         w    = combo_out[mask]
         peqn = geom.linear_model.fit_weighted(filtered_points, w)
         geom.linear_model.normal_towards_origin(peqn)
         -- DONE score with 2D
         -- plane_finder.util.score_plane_with_normal_threshold(
         new_s, new_x, new_c, new_n_pts = 
            plane_finder.util.score_plane_2D(
               peqn, points, normals, score_res_thres,  score_norm_thres, n_measurements)
         
         if new_s > prev_s then
            printf("[%d][%d] score old: %f (%d) new: %f (%d) orig: %f (%d)",
                   pi,li,prev_s,prev_n_pts,new_s,new_n_pts,orig_s,orig_n_pts)
            table.insert(curves, {string.format("iteration: %d thres: %2.1f mm norm: %0.3f rad",
                                                li, res_scale,nrm_scale),new_c})
            print(" - updating plane")
            prev_s = new_s
            prev_c = new_c
            prev_n_pts = new_n_pts
            p.eqn = peqn
            -- recompute if we change the equation
            residuals = geom.linear_model.residual_fast(points,p.eqn)
            inum = string.format("%03d_%03d", pi, li)
            score = string.format("s%f_r%2.1f_n%0.3f",new_s,res_scale,nrm_scale)
            image.save(string.format("%s_%s_residual.png",inum,score),    image.combine(residual_out))
            image.save(string.format("%s_%s_normal.png",inum,score),      image.combine(normal_out))
            image.save(string.format("%s_%s_neighborhood.png",inum,score),image.combine(combo_out))
         elseif doresidual and (res_scale > 1) then 
            res_scale = res_scale * 0.7
            printf("Narrowing nbd : res %f mm", res_scale)
            doresidual = not doresidual
         elseif (nrm_scale > 0.008) then 
            nrm_scale = nrm_scale * 0.7
            printf("Narrowing nbd : normal %f radians", nrm_scale)
            doresidual = not doresidual
         else
            break
         end
      end
   end
   for i,c in pairs(curves) do 
      gnuplot.pngfigure(string.format("%03d_%03d_plot.png",pi,i))
      gnuplot.title(c[1])
      gnuplot.xlabel("residual threshold in mm")
      gnuplot.ylabel("normal threshold in radians")
      gnuplot.zlabel("number of points withing scoring thresholds")
      -- gnuplot.raw("set key bottom right")
      gnuplot.splot(c[2])
      gnuplot.close()
   end
end
