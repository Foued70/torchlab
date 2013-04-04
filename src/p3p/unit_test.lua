util = require 'util'
geom = util.geom
p3p = require "p3p"

-- receate the tests from Kneip's paper
-- 1000 random points in 4x4x4
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
-- camera at 0,6,0
c = {
   xyz = torch.Tensor({0,6,0}),
   -- camera rotation
   rot = torch.Tensor({{1,0,0},{0,-1,0},{0,0,1}}),
   -- image sizes from Kniep paper
   w = 640,
   h = 480,
   -- center
   cw = 320,
   ch = 240,
   -- focal
   f = 800 
}
c.fovx = math.atan2(c.cw,c.f)
c.fovy = math.atan2(c.ch,c.f)
c.quat = geom.rotation_matrix_to_quaternion(c.rot)
c.quat_r = geom.quat_conjugate(c.quat)

function c:global2local(v)
   return geom.rotate_by_quat(v - self.xyz, self.quat_r)
end
   
function c:globalxyz2angle(pt)
   local v = self:global2local(pt)
   return torch.Tensor({torch.atan2(v[2],v[1]), torch.asin(v[3])})
end

rot_error = 0 
trans_error = 0 

for i = 1,10 do 
   -- pick 3 random points
   pts = pts3D[torch.LongTensor():randperm(npts):narrow(1,1,3)]
   angles = torch.Tensor(3,2)
   angles[1] = c:globalxyz2angle(pts[1])
   angles[2] = c:globalxyz2angle(pts[2])
   angles[3] = c:globalxyz2angle(pts[3])
   
   -- project vector onto camera plane 
   for i = 1,3 do 
      printf("pts[%d](%f,%f,%f) angle: %f %f",i,
          pts[i][1],pts[i][2],pts[i][3],angles[i][1],angles[i][2])
   end

   solutions = p3p.compute_poses(pts,angles)
   
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
         local crot_err = geom.quaternion_dist(c.quat,quat)
         if crot_err < rot_err then rot_err = crot_err end
         printf("+ Rotation (error: %f)", rot_err)
         printf(" -   p3p: %f %f %f %f norm: %f",quat[1],quat[2],quat[3],quat[4],quat:norm())
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
