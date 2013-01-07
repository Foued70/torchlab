require 'torch'
require 'sys'

require 'util'

util.test.pose = {}
local test  = util.test.pose
local pose  = util.pose
local geom  = util.geom

torch.include('util','pose-data.lua')

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
         local t   = pose.global2local(poses,i,gxyz[j]):narrow(1,1,3)
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
         local t     = torch.Tensor({pose.globalxyz2uv(poses,i,gxyz[j])})
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
   local e        = 0
   local maxerr = 0
   local cnt      = 0
   local poses    = test.data.poses
   local gxyz     = test.data.xyz
   sys.tic()
   for i = 1,poses.nposes do
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local gtxyz  = gxyz[j]      
         local t      = torch.Tensor({pose.globalxyz2uv(poses,i,gtxyz)})
         local pt,dir = pose.localxy2globalray(poses,i,t[3],t[4])
         -- print(string.format("%2.2f %2.2f", t[3],t[4]))
         local gdir   = geom.normalize(gtxyz - pt)
         local er     = torch.abs(dir:narrow(1,1,3) -gdir)
         local err, argerr = torch.max(er,1)
         err = err[1]
         argerr = argerr[1]
         if err > 1e-6 then
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

function test.all()
   test.global2local()
   test.globalxyz2uv()
   test.localxy2globalray()
end