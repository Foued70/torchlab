util       = require 'util'
geom       = util.geom
p3p        = require "p3p"
LensSensor = require "util.LensSensor"

-- receate the tests from Kneip's paper
-- 1000 random points in roughly 4x4x4
npts = 1000
pts3D = torch.randn(npts,3)
printf("pts3D range")
local x = pts3D[{{},1}]
local xmin = x:min()
local xmax = x:max()
local y = pts3D[{{},2}]
local ymin = y:min()
local ymax = y:max()
local z = pts3D[{{},3}]
local zmin = z:min()
local zmax = z:max()
printf(" - x: %f < x < %f",xmin,xmax)
printf(" - y: %f < y < %f",ymin,ymax)
printf(" - z: %f < z < %f",zmin,zmax)

axes  = torch.eye(3)
xaxis = axes[1]
yaxis = axes[2]
zaxis = axes[3]

-- using one of our lens objects
c = LensSensor.new("nikon_10p5mm_r2t_full",640,480)

-- camera at random point on sphere of radius 6
c.xyz = geom.normalize(torch.randn(3)) * 6

c.fovx   = math.atan2(c.center_x,c.hfocal_px)
c.fovy   = math.atan2(c.center_y,c.vfocal_px)

c.quat_r = geom.quaternion_from_to(xaxis,geom.normalize(torch.mul(c.xyz,-1)))
c.quat   = geom.quat_conjugate(c.quat_r)

function c:global2local(v)
   return geom.normalize(geom.rotate_by_quat(v - self.xyz, self.quat_r))
end
   
function c:globalxyz2angle(pt)
   local v = self:global2local(pt) 
   local angles = torch.Tensor({torch.atan2(v[2],v[1]), torch.asin(v[3])})
   printf("pt(%f,%f,%f) v(%f,%f,%f) angle: %f %f",
          pt[1],pt[2],pt[3],v[1],v[2],v[3],angles[1],angles[2])
   return angles
end

rot_error = 0 
trans_error = 0 

for i = 1,10 do 
   -- pick 3 random points
   pts = pts3D[torch.LongTensor():randperm(npts):narrow(1,1,3)]
   unit_vec = torch.Tensor(3,3)
   unit_vec[1] = c:global2local(pts[1])
   unit_vec[2] = c:global2local(pts[2])
   unit_vec[3] = c:global2local(pts[3])
   solutions = p3p.compute_poses(pts,unit_vec)
   
   print(unit_vec[1])
   print(geom.normalized(pts[1]))
   print(pts[1])
   print(c.quat_r)
   local trans_err = 1e16 
   local rot_err = 1e16 
   
   for ui = 1,4 do 
      s = solutions[ui]
      trans = s[1]
      rot   = s:narrow(1,2,3)
      quat  = geom.rotation_matrix_to_quaternion(rot)
      printf("Root: [%d]", ui)
      if quat then 
         local ctrans_err = torch.dist(c.xyz,trans)
         if ctrans_err < trans_err then trans_err = ctrans_err end
         printf("+ Position (error: %f)",ctrans_err)
         printf(" -   p3p: %f %f %f",trans[1],trans[2],trans[3])
         printf(" -   xyz: %f %f %f",c.xyz[1],c.xyz[2],c.xyz[3])
         local crot_err = geom.quaternion_dist(c.quat_r,quat)
         if crot_err < rot_err then rot_err = crot_err end
         printf("+ Rotation (error: %f)", rot_err)
         printf(" -   p3p: %f %f %f %f norm: %f",
                quat[1],quat[2],quat[3],quat[4],quat:norm())
         printf(" - c.quat_r: %f %f %f %f norm: %f",
                c.quat_r[1],c.quat_r[2],c.quat_r[3],c.quat_r[4],c.quat_r:norm())
      else
         print("skipping")
      end
   end
   printf("Lowest trans error: %f", trans_err)
   trans_error = trans_error + trans_err
   printf("Lowest rot error: %f", rot_err)
   rot_error = rot_error + rot_err 
end

printf("Total Translation Error: %f", trans_error)
printf("Total Rotation Error: %f", rot_error)
