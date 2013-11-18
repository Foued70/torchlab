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


itrw = plane_finder.IterativeReweightedFit.new{
   residual_thres = 100,
   residual_decr  = 0.7,
   residual_stop  = 1,
   normal_thres   = math.pi/3,
   normal_decr    = 0.7,
   normal_stop    = math.pi/360
}

-- itrw.save_images = true

for pi = 1,#pl do 
   p = pl[pi]
   itrw.image_id = pi
   best_plane, best_score, best_n_points, curves = itrw:fit(points, normals, p.eqn)

   gnuplot.pngfigure(string.format("%03d_plot.png",pi))
   gnuplot.xlabel("residual threshold in mm")
   gnuplot.ylabel("number of points withing scoring thresholds")
   gnuplot.raw("set key bottom right")   
   gnuplot.plot(curves)
   gnuplot.close()
end
