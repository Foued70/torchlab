require 'torch'
require 'sys'

local util  = require 'util'
local Poses = util.Poses
local geom  = util.geom

local test  = {}

test.data = require 'util/test/pose-data'

-- FIXME check serious numerical issues which are hopefully due to the
-- precision at which the groundtruth is copied from blender

function test.global2local ()
   print("Testing global2local") 
   local e      = 0
   local maxerr = 0
   local cnt    = 0
   local poses  = test.data.poses

   local gxyz   = test.data.xyz
   local result = test.data.result_global2local
   sys.tic()
   for i = 1,poses.nposes do
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local t   = poses:global2local(i,gxyz[j]):narrow(1,1,3)
         local gt  = result[i][j]
         local er  = torch.abs(gt - t)
         local err = torch.max(er)
         if err > maxerr then maxerr = err end
         if err > 1e-1 then 
            e = e + 1
            print(err)
            print(string.format("%e <-> %e : %e",t[1],gt[1],er[1]))
            print(string.format("%e <-> %e : %e",t[2],gt[2],er[2]))
            print(string.format("%e <-> %e : %e",t[3],gt[3],er[3]))
         end
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,cnt, maxerr,sys.toc()))
end

function test.globalxyz2uv ()
   print("Testing globalxyz2uv") 
   local e        = 0
   local maxuverr = 0
   local maxpxerr = 0
   local cnt      = 0
   local poses    = test.data.poses
   local gxyz     = test.data.xyz
   local result   = test.data.result_globalxyz2uv
   sys.tic()
   for i = 1,poses.nposes do
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local t     = torch.Tensor({poses:globalxyz2uv(i,gxyz[j])})
         local gt    = result[i][j]
         local er    = torch.abs(gt - t)
         local uverr = torch.max(er:narrow(1,1,2))
         local pxerr = torch.max(er:narrow(1,3,2))
         if uverr > maxuverr then maxuverr = uverr end
         if pxerr > maxpxerr then maxpxerr = pxerr end
         if uverr > 1e-2 or pxerr > 10 then 
            e = e + 1
            print(string.format("UV error: %e",uverr))
            print(string.format("%e <-> %e : %e",t[1],gt[1],er[1]))
            print(string.format("%e <-> %e : %e",t[2],gt[2],er[2]))
            print(string.format("PX error: %e",pxerr))
            print(string.format("%e <-> %e : %e",t[3],gt[3],er[3]))
            print(string.format("%e <-> %e : %e",t[4],gt[4],er[4]))
         end
      end
   end
   print(string.format(" - Found %d/%d errors (max uv: %e px: %e) in %2.4fs",
                       e,cnt, maxuverr,maxpxerr,sys.toc()))
end


function test.localxy2globalray ()
   print("Testing localxy2globalray") 
   local e      = 0
   local maxerr = 0
   local cnt    = 0
   local poses  = test.data.poses

   local gxyz   = test.data.xyz
   sys.tic()
   for i = 1,poses.nposes do
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local gtxyz  = gxyz[j]
         local t      = torch.Tensor({poses:globalxyz2uv(i,gtxyz)})
         local pt,dir = poses:localxy2globalray(i,t[3],t[4])
         local gdir   = geom.normalize(gtxyz - pt)
         local er     = torch.abs(dir:narrow(1,1,3) -gdir)
         local err, argerr = torch.max(er,1)
         err = err[1]
         argerr = argerr[1]
         if err > 2e-3 then
            e = e + 1
            print(string.format(" dir: %f, %f, %f",
                                dir[1],dir[2],dir[3]))
            print(string.format("gdir: %f, %f, %f",
                                gdir[1],gdir[2],gdir[3]))
            print(string.format("   %d: %f, %f, %f", 
                                argerr, er[1],er[2],er[3]))
         end
         if err > maxerr then maxerr = err end
      end 
   end
   print(string.format(" - Found %d/%d errors (max %e ) in %2.4fs",
                       e,cnt, maxerr,sys.toc()))
end

-- tests 2globalray in context of poses
function test.localxy2globalray_pose ()
   print("Testing localxy2globalray_poses") 
   local poses  = test.data.poses
   -- matterport textures go beyond 360 
   local over = torch.floor((poses.w - poses.px[1] * 1/360)*0.5 + 0.5)
   for pi = 1,poses.nposes do 
      local yerr = 0
      local xerr = 0
      local tot  = 0
      local w  = poses[pi].w
      local h  = poses[pi].h
      for y = 1,h,100 do 
         for x = over[pi],w-over[pi],100 do 
            local pt, dir = poses:localxy2globalray(pi, x, y)
            local r = Ray(pt,dir)
            local vec = r(10)
            local u,v,nx,ny = poses:globalxyz2uv(pi, vec)
            local nx = math.floor(nx)
            local ny = math.floor(ny) 
            if (y ~= ny) then
               yerr = yerr + 1
            end
            if (x ~= nx) then
               xerr = xerr + 1
            end
            tot = tot + 1
         end
      end
      printf(" - [%d] Errors: x:%d y:%d both: %d/%d", 
             pi, xerr, yerr, xerr + yerr, tot)
   end
end 

function test.all()
   test.global2local()
   test.globalxyz2uv()
   test.localxy2globalray()
   test.localxy2globalray_pose()
end


return test
