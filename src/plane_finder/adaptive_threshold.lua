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

rout = io.open('residual.txt', 'w')

-- plot residual
for i = 1,#pl do 
   p = pl[i]
   p.res = geom.linear_model.residual(test_points,p.eqn)
   p.res:abs()

   p.res_cum = torch.Tensor(2,100)
   r = torch.linspace(1,100)
   p.res_cum[1] = r
   p.res_d1 = torch.zeros(100)
   for j = 1,100 do 
      thres = r[j]
      curr = p.res:lt(thres):sum()
      p.res_cum[2][j] = curr
      if j > 2 then 
         p.res_d1[j-1] = p.res_cum[{2,j}] - p.res_cum[{2,j-2}]
      end

   end
   p.res:resize(map_height, map_width)
   image.save(string.format("residual_plane_%03d.png",i),image.combine(torch.log(p.res))) 
   image.save(string.format("residual_plane_%03d_thres_%2.4f.png",i,r[15]),image.combine(p.res:lt(r[15])))
   for j = 1,100 do 
      w = 0.01 + math.abs(j-15)/100
      rout:write(string.format("%d %f %f %f %f\n", 
                               j, p.res_cum[1][j], p.res_cum[2][j], p.res_d1[j], p.res_d1[j] * w))
   end
   rout:write("\n\n")
end

rout:close()


rout = io.open('normals.txt', 'w')

-- plot residual
for i = 1,#pl do 
   p = pl[i]
   p.diff = allnrm - p.eqn[{{1,3}}]:resize(1,3):expandAs(allnrm)
   p.dist = p.diff:cmul(p.diff):sum(2):squeeze() -- :sqrt()
   p.nrm_cum = torch.Tensor(2,100)
   r = torch.linspace(0,1)
   p.nrm_cum[1] = r
   p.nrm_d1 = torch.zeros(100)
   for j = 1,100 do 
      thres = r[j]
      curr = p.dist:lt(thres):sum() 
      p.nrm_cum[2][j] = curr
      if j > 2 then 
         p.nrm_d1[j-1] = p.nrm_cum[{2,j}] - p.nrm_cum[{2,j-2}]
      end
   end
   p.dist:resize(map_height, map_width)
   image.save(string.format("normalDist_plane_%03d.png",i),image.combine(p.dist))
   image.save(string.format("normalDist_plane_%03d_thres_%2.4f.png",i,r[15]),image.combine(p.dist:lt(r[15])))
   for j = 1,100 do 
      w = 0.01 + math.abs(j-15)/100
      rout:write(string.format("%d %f %f %f %f\n", 
                               j, p.nrm_cum[1][j], p.nrm_cum[2][j], p.nrm_d1[j], p.nrm_d1[j] * w))
   end
   rout:write("\n\n")
end

rout:close()
