
--   -- idea: projecting whole wireframe onto pose would help with
--      alignment of all faces at once (Reduce human alignment
--      tweaks).
--
-- thought to help debug the image stitching: overlay the wireframe of
-- the regeom on the texture.
function draw_wireframe (p,i,obj)
   local pimage = p.images[i]
   local psize = pimage:size()
   psize[1] = 4
   local wimage = torch.Tensor(psize):fill(0)
   local face_verts = obj.face_verts
   local nverts = obj.nverts_per_face
   for fi = 1,face_verts:size(1) do
      local nv = nverts[fi]
      local f = face_verts[fi]:narrow(1,1,nv)
      local pvert = f[nv]
      for vi = 1,nv do
         local cvert = f[vi]
         local dir = cvert - pvert
         local len = torch.norm(dir)
         if (len > 1e-8) then
            dir = dir/len
            step = dir * mpp
            -- printf("step: %f,%f,%f",step[1],step[2],step[3])
            for s = 0,len,mpp do
               -- draw verts first
               local u,v,x,y = util.pose.globalxyz2uv(p,i,pvert)
               -- printf("u: %f v: %f x: %f y %f", u, v, x, y)
               if (u > 0) and (u < 1) and (v > 0) and (v < 1) then
                  wimage[{1,y,x}] = 1  -- RED
                  wimage[{4,y,x}] = 1  -- Alpha Channel
               end
               pvert = pvert + step

            end
         end
      end
   end
   return wimage
end

function save_all_wireframes()
   for pi = 1,poses.nposes do
      local wimage = draw_wireframe(poses,pi,target)
      image.display(wimage)
      -- save
      local wimagename = poses[pi]:gsub(".jpg","_wireframe.png")
      printf("Saving: %s", wimagename)
      image.save(wimagename,wimage)
   end
end
