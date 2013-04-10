require 'image'

local geom = util.geom
local p3p = require 'p3p'
local LensSensor = util.LensSensor


-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute Perspective 3 points algorithm')
cmd:text()
cmd:text('Options')
cmd:option('-posefile',
           -- '../data/test/texture_swap/scanner371_job224000_texture_info.txt',
           "../data/models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt",
           'pose info file in same directory as the texture images')
cmd:option('-outdir','output/')
cmd:text()

-- parse input params
params = cmd:parse(arg)

-- temp load data from ui
data = dofile("Kitchen_calibration_4_3_13.lua")

img = image.load(data[1][1].image_path)

-- load lens calibration
lens = LensSensor.new("nikon_10p5mm_r2t_full",img)
-- lens = LensSensor.new("nikon_D5100_w10p5mm",img)

rot_error = 0 
trans_error = 0 

for si,sweep in pairs(data) do 
   -- matterport pose
   pp = sweep[1].pose_position
   pr = sweep[1].pose_rotation
         
   for pi,photo in pairs(sweep) do 
      printf("Sweep:%d Photo:%d", si, pi)
      -- matterport + offset
       hp = sweep[pi].photo_position
       hr = sweep[pi].photo_rotation

      if photo.white_wall then
         print("white wall flag skipping")
      else
         if not photo.world then 
            photo.world = photo.calibration_pairs:narrow(2,1,3)
            photo.uv    = photo.calibration_pairs:narrow(2,4,2)
            photo.uc    = lens:img_coords_to_world(photo.uv,"uv","uc")
         end

         solutions = p3p.compute_poses(photo.world,photo.uc)

         local trans_err = 1e16 
         local rot_err = 1e16 
         for ui = 1,4 do 
            s = solutions[ui]
            trans = s[1]
            rot   = s:narrow(1,2,3)
            quat  = geom.rotation_matrix_to_quaternion(rot)
            printf("Root: [%d]", ui)
            if quat then 
               local ctrans_err = torch.dist(pp,trans)
               if ctrans_err < trans_err then trans_err = ctrans_err end
               printf("+ Position (error: %f)",ctrans_err)
               printf(" - sweep: %f %f %f",pp[1],pp[2],pp[3])
               printf(" - photo: %f %f %f",hp[1],hp[2],hp[3])
               printf(" -   p3p: %f %f %f",trans[1],trans[2],trans[3])
               local crot_err = geom.quaternion_dist(pr,quat)
               if crot_err < rot_err then rot_err = crot_err end
               printf("+ Rotation (error: %f)", rot_err)
               printf(" - sweep: %f %f %f %f norm: %f",pr[1],pr[2],pr[3],pr[4],pr:norm())
               printf(" - photo: %f %f %f %f norm: %f",hr[1],hr[2],hr[3],hr[4],hr:norm())
               printf(" -   p3p: %f %f %f %f norm: %f",quat[1],quat[2],quat[3],quat[4],quat:norm())

               -- new unit vectors from new position should align with the 
               newuc = torch.Tensor(photo.uc:size())
               for uci = 1,newuc:size(1) do 
                  newuc[uci] = 
                     geom.normalize(geom.rotate_by_quat(photo.world[uci] - trans, 
                                                        geom.quat_conjugate(quat)))
               end
               printf("angle dist: %f", newuc:dist(photo.uc))
               print(newuc)
               print(photo.uc)
            else
               print("skipping")
            end
         end
         printf("Lowest trans error: %f", trans_err)
         trans_error = trans_error + trans_err
         printf("Lowest rot error: %f", rot_err)
         rot_error = rot_error + rot_err
      end
   end
end
printf("Total Translation Error: %f", trans_error)
printf("Total Rotation Error: %f", rot_error)