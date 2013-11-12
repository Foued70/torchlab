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

_G.xyz_map   = pc:get_xyz_map()
_G.map_height = xyz_map:size(2)
_G.map_width  = xyz_map:size(3)
_G.points     = xyz_map:reshape(xyz_map:size(1),map_height*map_width)
_G.points_NxD = points:t():contiguous()

_G.normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
normals:resize(3,map_height*map_width)
_G.normals_NxD = normals:t():contiguous()

p = pl[1]

_G.sig = nn.Sigmoid()

res_scale = 100
nrm_scale = 0.4

p.residuals = geom.linear_model.residual(points_NxD,p.eqn)

-- sort of kernel. Could learn thresholds for planes and normals
-- except that we don't have to because there are plausible real world thresholds
_G.x = p.residuals:clone()
x:div(res_scale)
x:resize(map_height,map_width)
_G.residual_out = sig:forward(x):clone()

-- invert abs of sigmoid (0 -> 1)
residual_out:add(-0.5):abs():mul(-2):add(1)

_G.normal_diff = plane_finder.util.compute_normal_diff(p.eqn, normals_NxD)
normal_diff:div(nrm_scale)
normal_diff:resize(map_height,map_width)
_G.normal_out = sig:forward(normal_diff):clone()
-- invert abs of sigmoid (0 -> 1)
normal_out:add(-0.5):abs():mul(-2):add(1)

-- TODO some mixing ? alpha = 0.5
combo = residual_out:clone()
combo:cmul(normal_out)

ss = nn.SoftShrink(0.7)
_G.combo_out = ss:forward(combo):clone()

-- batch with neighborhood distance

mask = combo_out:gt(0)

filtered_points = torch.Tensor(3,mask:sum())
for i = 1,3 do 
   filtered_points[i] = points[i][mask] 
end
w = combo_out[mask]
peqn = geom.linear_model.fit_weighted(filtered_points, w)

s,c = plane_finder.util.score_plane(pl[1].eqn,points_NxD, 100)
snew, cnew = plane_finder.util.score_plane(peqn,points_NxD, 100)

print("score old: "..s.." new: "..snew)
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("wxt")
gnuplot.plot({{c:t()},{cnew:t()}})
