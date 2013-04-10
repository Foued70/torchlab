util       = require 'util'
geom       = util.geom
p3p        = require "p3p"
LensSensor = util.LensSensor

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

-- make camera point at origin
view_direction = geom.normalize(torch.mul(c.xyz,-1))
c.quat_r = geom.quaternion_from_to(xaxis,view_direction)
c.quat   = geom.quat_conjugate(c.quat_r)

-- copy these functions from what used to be Pose.lua
function c:local2global(v)
   return geom.rotate_by_quat(v,self.quat) + self.xyz
end

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

function check_solutions(solutions,c,debug)
   local trans_err = 1e16
   local rot_err = 1e16
   local reprojection_err = 1e16
   local best_solution = 0
   local best_soln_trans = 1e16
   local best_soln_rot   = 1e16
   for ui = 1,4 do
      s = solutions[ui]
      trans = s[1]
      rot   = s:narrow(1,2,3)
      quat  = geom.rotation_matrix_to_quaternion(rot)
      if debug then
         printf("Root: [%d]", ui)
      end
      if quat then
         local ctrans_err = torch.dist(c.xyz,trans)
         if ctrans_err < trans_err then trans_err = ctrans_err end
         local crot_err = geom.quaternion_dist(c.quat_r,quat)
         if crot_err < rot_err then rot_err = crot_err end

         -- new unit vectors from new position should align with the original 
         newuc = torch.Tensor(3,3) -- c.uc:size())
         for uci = 1,3 do -- newuc:size(1) do 
            newuc[uci] = 
               geom.normalize(geom.rotate_by_quat(c.world[uci] - trans, quat))
         end
         -- the reprojection error on the three points will solve the 
         local creproj_err = newuc:dist(c.uc:narrow(1,1,3))

         if creproj_err < reprojection_err then 
            best_solution = ui
            best_soln_trans = ctrans_err
            best_soln_rot   = crot_err
            reprojection_err = creproj_err 
         end

         if debug then 
            printf("+ Position (error: %f)",ctrans_err)
            printf(" -   p3p: %f %f %f",trans[1],trans[2],trans[3])
            printf(" -   xyz: %f %f %f",c.xyz[1],c.xyz[2],c.xyz[3])
            printf("+ Rotation (error: %f)", crot_err)
            printf(" -   p3p: %f %f %f %f norm: %f",
                   quat[1],quat[2],quat[3],quat[4],quat:norm())
            printf(" - c.quat_r: %f %f %f %f norm: %f",
                   c.quat_r[1],c.quat_r[2],c.quat_r[3],c.quat_r[4],c.quat_r:norm())

            printf("+ Reprojection (error: %f)", creproj_err)
         end
      else
         if debug then 
            print("skipping")
         end
      end
   end
   if debug then
      if (trans_err ~= best_soln_trans) then 
         printf("best solution not best translation: %f <> %f",
            trans_err, best_soln_trans)
      end
      if (rot_err ~= best_soln_rot) then 
         printf("best solution not best rotation: %f <> %f",
                rot_err, best_soln_rot)
      end
   end
   return trans_err, rot_err, reprojection_err, best_solution
end


project_through_corners = false
jitter_image_coords     = false -- true
jitter_amount           = 0 -- 1e-2
sample_size             = 1000

debug_solutions = false

for ji = -5,0,0.1 do 
   jitter_amount = 10 ^ ji

   rot_error    = torch.Tensor(sample_size)
   trans_error  = torch.Tensor(sample_size)
   reproj_error = 0
   error_i = 0
   
   for i = 1,sample_size do
      -- reset camera position
      c.xyz = geom.normalize(torch.randn(3)) * 6
      c.quat_r = geom.quaternion_from_to(xaxis,geom.normalize(torch.mul(c.xyz,-1)))
      c.quat   = geom.quat_conjugate(c.quat_r)
      if true then
         if project_through_corners then 
            -- shoot rays through corners of camera, put in world coords
            test_corners = torch.Tensor({{-1,-1},{-1,1},{1,-1},{1,1}})
         else
            -- shoot rays through random points in image
            test_corners = torch.rand(4,2):mul(2):add(-1)
         end
         test_unit    = c:img_coords_to_world(test_corners,"uv","uc")
         test_world   = test_unit:clone()
         -- project random distances in front of camera
         for ti = 1,test_unit:size(1) do
            test_world[ti]:mul(math.random()*6)
            test_world[ti] = c:local2global(test_world[ti])
         end 
         -- jitter
         if jitter_image_coords then 
            test_corners:add(torch.randn(test_corners:size()):mul(jitter_amount))
            test_unit    = c:img_coords_to_world(test_corners,"uv","uc") 
         else
            -- jitter unit vectors directly
            test_unit:add(torch.randn(test_unit:size()):mul(jitter_amount))
            for ti = 1,test_unit:size(1) do
               geom.normalize(test_unit[ti])
            end
         end
         
         c.world = test_world
         c.uc    = test_unit 
         
         solutions = p3p.compute_poses(test_world,test_unit)
         
      else 
         -- pick 3 random points as in paper
         pts = pts3D[torch.LongTensor():randperm(npts):narrow(1,1,3)]
         unit_vec    = torch.Tensor(3,3)
         unit_vec[1] = c:global2local(pts[1])
         unit_vec[2] = c:global2local(pts[2])
         unit_vec[3] = c:global2local(pts[3])
         c.world = pts
         c.uc    = unit_vec
         solutions   = p3p.compute_poses(pts,unit_vec)
      end
      if solutions then 
         local trans_err, rot_err, reproj_err, bs = check_solutions(solutions,c,debug_solutions)
         if bs > 0 then
            error_i = error_i + 1
            trans_error[error_i]  = trans_err
            rot_error[error_i]    = rot_err
            reproj_error = reproj_error + reproj_err
            if print_sample_error then 
               printf("Lowest trans error : %f", trans_err)
               printf("Lowest rot error   : %f", rot_err)
               printf("Lowest reproj error: %f", reproj_err)
            end
         end
      end
   end
   trans_error = trans_error:narrow(1,1,error_i)
   rot_error = rot_error:narrow(1,1,error_i):mul(1/3) -- average error per vector
   printf("%e trans: %e %e %e %e rot: %e %e %e %e skipped: %2.2f", 
          jitter_amount, 
          trans_error:mean(), trans_error:std(),trans_error:min(), trans_error:max(),
          rot_error:mean(), rot_error:std(), rot_error:min(), rot_error:max(),
          1 - (error_i / sample_size))
end