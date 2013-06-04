require 'image'

local LensSensor = projection.LensSensor
local p3p        = require 'p3p'


-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute Perspective 3 points algorithm')
cmd:text()
cmd:text('Options')
cmd:option('-data_file',
           "picking_data/Kitchen_calibration_4_3_13.lua",
           'pose info file in same directory as the texture images')
cmd:option("-lens_type", "nikon_10p5mm_r2t_full","data for image dewarping")

cmd:text()

-- parse input params
params = cmd:parse(arg)

-- temp load data from ui
data = dofile(params.data_file)

img = image.load(data[1][1].image_path)

-- load lens calibration
lens = LensSensor.new(params.lens_type,img)

rot_error   = 0 -- torch.Tensor(3)
trans_error = 0 -- torch.Tensor(3)

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
            photo.uc    = lens:image_coords_to_angles(photo.uv,"uv","uc")
         end

         solutions = p3p.compute_poses(photo.world,photo.uc)

         local trans_err = 1e16 
         local rot_err = 1e16 
         for ui = 1,4 do 
            s = solutions[ui]
            trans = s[1]
            rot   = s:narrow(1,2,3)
            quat  = geom.quaternion.from_rotation_matrix(rot)
            printf("Root: [%d]", ui)
            if quat then 
               local ctrans_err = torch.dist(pp,trans)
               if ctrans_err < trans_err then trans_err = ctrans_err end
               printf("+ Position (error: %f)",ctrans_err)
               printf(" - sweep: %f %f %f",pp[1],pp[2],pp[3])
               printf(" - photo: %f %f %f",hp[1],hp[2],hp[3])
               printf(" -   p3p: %f %f %f",trans[1],trans[2],trans[3])
               local crot_err = geom.quaternion.distance(pr,quat)
               if crot_err < rot_err then rot_err = crot_err end
               printf("+ Rotation (error: %f)", rot_err)
               printf(" - sweep: %f %f %f %f norm: %f",pr[1],pr[2],pr[3],pr[4],pr:norm())
               printf(" - photo: %f %f %f %f norm: %f",hr[1],hr[2],hr[3],hr[4],hr:norm())
               printf(" -   p3p: %f %f %f %f norm: %f",quat[1],quat[2],quat[3],quat[4],quat:norm())

               -- new unit vectors from new position should align with the 
               newuc = torch.Tensor(photo.uc:size())
               for uci = 1,newuc:size(1) do 
                  newuc[uci] = 
                     geom.util.normalize(geom.quaternion.rotate(photo.world[uci] - trans, 
                                                                geom.quaternion.conjugate(quat)))
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