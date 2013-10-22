plane_finder = require './plane_finder.lua'

if not pc then
   src_dir = 'arcs/temporary-circle-6132/source/po_scan/a/001/'
   wrk_dir = src_dir:gsub("source","work")

   -- new scans are in mm which makes most default measurements 1000x off...
   max_radius      = 25000

   pcfile = src_dir .. 'sweep.xyz'

   _G.pc   = PointCloud.PointCloud.new(pcfile, max_radius)
   _G.normals,dd,norm_mask = pc:get_normal_map()

   xyz_map = pc:get_xyz_map()
   _G.allpts  = xyz_map:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()
end
if not planes then
   _G.planes = torch.load("planes.t7")
end

function find_rotation_to_align_axis(norm)
   local angle_az = geom.util.unit_cartesian_to_spherical_angles(planes[3].eq)
   -- TODO check this again.  Disconnect btw. 2D projections and 3D.  I would have expected azimuth in first
   return geom.quaternion.from_euler_angle(torch.Tensor({0,angle_az[1]}))
end

function axis_align(pts,quat)
   quat = quat or find_rotation_to_align_axis(pts)
   return geom.quaternion.rotate(quat,pts), quat
end

function _G.dump_pts(pts,fname)
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
function align_planes(planes,allpts)
   maxv = -1
   midx = 1
   for i,p in pairs(planes) do
      if (p.eq[1] > maxv) and (p.npts > allpts:size(1)*0.01) then
         maxv = p.eq[1] ;
         midx = i ;
      end
   end

   pm = planes[midx]
   quat = find_rotation_to_align_axis(pm.eq)
   aapts = axis_align(allpts,quat)
   return quat, aapts
end

function aligned_planes_to_obj (planes, aapts, quat, fname)
   io = require 'io'
   fname = fname or "planes.obj"

   objf = io.open(fname,'w')

   -- loop through planes. rotate normals.  find bbx in plane. create rectangle.
   for i,p in pairs(planes) do

      ppts = plane_finder.select_points(aapts,p.mask)
      dump_pts(ppts,string.format("plane_%d_pts.xyz",i))

      pn = geom.quaternion.rotate(quat,p.eq)
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

quat, aapts = align_planes(planes,allpts)
aligned_planes_to_obj (planes, aapts, quat, fname)
