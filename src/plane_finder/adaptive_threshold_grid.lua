io = require 'io'
plane_finder = require './plane_finder'
src_dir = "arcs/motor-unicorn-0776/source/faro/"
-- s = src_dir .. "sweep_001.xyz"
s = "arcs/temporary-circle-6132/source/po_scan/a/001/sweep.xyz"
_G.pc = PointCloud.PointCloud.new(s)

wrk_dir = "arcs/motor-unicorn-0776/work/"
trans_dir = wrk_dir .. "transf_for_marco/"

_G.t = torch.load(trans_dir .. "transf_sweep_001_sweep_001.dat")

pc:set_pose_from_rotation_matrix(t)
_G.pc_points  = pc:get_global_points()
pc_pose       = pc:get_global_scan_center()
pc_max_radius = pc:get_max_radius()

-- pname = "combination__thres_40_minseed_150_minplane_900_nf_0.87_normal_raw/planes.t7"
-- _G.pl = torch.load(wrk_dir .. "planes/sweep_001/".. pname)

_G.pl = torch.load("output/arcs_temporary-circle-6132_source_po_scan_a_001_sweep_xyz/combined__thres_40_minplane_150_nf_0.87_normal_var_baseplanes_saliency_base_12_scale_1.2_n_scale_5_thres_20_minseed_150_minplane_900_nf_0.87_normal_var/planes.t7")

-- _G.pls = plane_finder.Planes.new(pl)

xyz_map   = pc:get_xyz_map()
map_height = xyz_map:size(2)
map_width  = xyz_map:size(3)
points     = xyz_map:reshape(xyz_map:size(1),map_height*map_width):t():contiguous()
_G.test_points = points

threshold                      = 40
normal_filter                  = true
normal_threshold               = math.cos(math.pi/30)
_G.normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()

allnrm = torch.Tensor(3,xyz_map:size(2)*xyz_map:size(3))
allnrm[{{1,3},{}}]:copy(normals)
-- allnrm[{4,{}}]:copy(dd)
allnrm = allnrm:t():contiguous()

_G.depth_map = torch.zeros(map_height, map_width)
_G.depth_map_adapt = torch.zeros(map_height, map_width)


function compute_normal_dist(plane_eqn, normals)
   local diff = normals - plane_eqn[{{1,3}}]:resize(1,3):expandAs(normals)
   return diff:cmul(diff):sum(2):squeeze():sqrt()
end

n_secondary_thres = 15

-- plot residual
for i = 1,#pl do
   ifile_name = string.format('plot_normal_dist_%03d_residual_thresholds.png',i)
   while true do
      if    util.fs.is_file(ifile_name) then
         break
      end
      p = pl[i]
      p.res_cum = torch.Tensor(1+n_secondary_thres,100)
      r = torch.linspace(1,100)
      p.res_cum[1] = r

      p.residuals = geom.linear_model.residual(points,p.eqn)
      p.residuals:abs()

      p.dist = compute_normal_dist(p.eqn, allnrm)

      -- try thresholded
      tcount = 1
      -- TODO torch.logspace is f'ed
      -- nthres is in cartesian normal space (max distance is 2 +1,-1)
      normal_thres = torch.linspace(-9,1,n_secondary_thres)
      normal_thres:apply(function (x) return 2^x end)
      for ni = 1,n_secondary_thres do
         collectgarbage()
         mask  = norm_mask:eq(0):cmul(p.dist:lt(normal_thres[ni]))
         residual_filtered = p.residuals[mask]

         if residual_filtered:nElement() > 0 then
            for j = 1,100 do
               thres = r[j]
               curr = residual_filtered:lt(thres):sum()
               p.res_cum[tcount+1][j] = curr
            end
            out = torch.Tensor(map_height * map_width):fill(residual_filtered:max()+1)
            out[mask:reshape(mask:nElement())] = residual_filtered
            out:resize(map_height,map_width)
            image.save(
               string.format("residual_plane_%03d_nthres_%f.png",
                             i,normal_thres[ni]),
               image.combine(torch.log1p(out)))
            image.save(
               string.format("residual_plane_%03d_nthres_%f_thres_%2.4f.png",
                             i,normal_thres[ni],r[15]),
               image.combine(out:gt(r[15])))
         else
            p.res_cum[tcount+1]:fill(0)
         end

         tcount = tcount + 1

      end
      dfile_name = string.format('data_residual_plane_%03d.txt',i)
      pfile_name = string.format('plot_residual_plane_%03d.gnu',i)
      ifile_name = string.format('plot_residual_plane_%03d_normal_thresholds.png',i)
      print("writing ")
      print("  + data       : "..dfile_name)
      print("  + plot_script: "..pfile_name)
      print("  + plot       : "..ifile_name)
      rout = io.open(dfile_name, 'w')
      pout = io.open(pfile_name, 'w')
      pout:write("set term png\n")
      pout:write("set key bottom right\n")
      pout:write("set xlabel 'distance to plane in millimeters'\n")
      pout:write("set ylabel 'normalized number of points within threshold'\n")
      pout:write(string.format("set output '%s'\n",ifile_name))
      pstr = string.format("plot '%s' \\\n",dfile_name)
      for t = 2,n_secondary_thres+1 do
         n_pts = p.res_cum[t][100]
         -- normalize
         p.res_cum[t]:div(n_pts)
         aoc = p.res_cum[t]:sum()/100
         normal_data_str = string.format("normal thres: %2.3f n_pts: %6d area: %1.3f", normal_thres[t-1], n_pts,aoc)
         rout:write("# " .. normal_data_str .."\n")
         for j = 1,100 do
            rout:write(string.format("%f %f\n",p.res_cum[1][j],p.res_cum[t][j]))
         end
         rout:write("\n\n")

         if n_pts > 100 then
            pout:write(string.format("%s index %d w l t '%s'",pstr,t-2,normal_data_str))
            pstr = " '' "
            if t < n_secondary_thres+1 then
               pout:write(",\\")
            end
            pout:write("\n")
         end
      end
      rout:close()
      pout:close()
      os.execute("gnuplot "..pfile_name)

      -- normals
      tcount = 1
      p.nrm_cum = torch.Tensor(1+n_secondary_thres,100)
      r = torch.linspace(0.001,2)
      residual_thres = torch.linspace(0,10,n_secondary_thres)
      residual_thres:apply(function (x) return 2^x end)
      p.nrm_cum[1] = r

      for ni = 1,n_secondary_thres do
         collectgarbage()
         mask  = norm_mask:eq(0):cmul(p.residuals:lt(residual_thres[ni]))
         normal_dist_filtered = p.dist[mask]

         if normal_dist_filtered:nElement() > 0 then

            for j = 1,100 do
               thres = r[j]
               curr = normal_dist_filtered:lt(thres):sum()
               p.nrm_cum[tcount+1][j] = curr
            end

            out = torch.Tensor(map_height * map_width):fill(normal_dist_filtered:max()+1)
            out[mask:reshape(mask:nElement())] = normal_dist_filtered
            out:resize(map_height,map_width)
            image.save(
               string.format("normal_dist_%03d_nthres_%f.png",
                             i,residual_thres[ni]),
               image.combine(torch.log1p(out)))
            image.save(
               string.format("normal_dist_%03d_nthres_%f_thres_%2.4f.png",
                             i,residual_thres[ni],r[15]),
               image.combine(out:gt(r[15])))
         else
            p.res_cum[tcount+1]:fill(0)
         end

         tcount = tcount + 1

      end
      dfile_name = string.format('data_normal_dist_%03d.txt',i)
      pfile_name = string.format('plot_normal_dist_%03d.gnu',i)
      ifile_name = string.format('plot_normal_dist_%03d_residual_thresholds.png',i)
      print("writing ")
      print("  + data       : "..dfile_name)
      print("  + plot_script: "..pfile_name)
      print("  + plot       : "..ifile_name)
      rout = io.open(dfile_name, 'w')
      pout = io.open(pfile_name, 'w')
      pout:write("set term png\n")
      pout:write("set key bottom right\n")
      pout:write("set xlabel 'distance to plane normal in cartesian coords'\n")
      pout:write("set ylabel 'normalized number of points within threshold'\n")
      pout:write(string.format("set output '%s'\n",ifile_name))
      pstr = string.format("plot '%s' \\\n",dfile_name)
      for t = 2,n_secondary_thres+1 do
         n_pts = p.nrm_cum[t][100]
         -- normalize
         p.nrm_cum[t]:div(n_pts)
         aoc = p.nrm_cum[t]:sum()/100
         normal_data_str = 
            string.format("residual thres: %2.3f n_pts: %6d area: %1.3f", residual_thres[t-1], n_pts,aoc)
         rout:write("# " .. normal_data_str .."\n")
         for j = 1,100 do
            rout:write(string.format("%f %f\n",p.nrm_cum[1][j],p.nrm_cum[t][j]))
         end
         rout:write("\n\n")

         if n_pts > 100 then
            pout:write(string.format("%s index %d w l t '%s'",pstr,t-2,normal_data_str))
            pstr = " '' "
            if t < n_secondary_thres+1 then
               pout:write(",\\")
            end
            pout:write("\n")
         end
      end
      rout:close()
      pout:close()
      os.execute("gnuplot "..pfile_name)
      break
   end
end
