local path = require 'path'
local Ray  = geom.Ray
local geom = geom.util

local test  = {}
test.data = require 'model.test.data.photo-data'
scan = model.mp.scan(path.join(__dirname, 'data'))
photos = scan:get_photos()

-- FIXME check serious numerical issues which are hopefully due to the
-- precision at which the groundtruth is copied from blender

function test.global2local ()
   print("Testing global2local") 
   local e      = 0
   local maxerr = 0
   local cnt    = 0

   local gxyz   = test.data.global_positions
   local result = test.data.result_global2local
   log.tic()
   for i = 1, #photos do
      local photo = photos[i]      
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local t   = photo:global2local(gxyz[j]):narrow(1,1,3)
         local gt  = result[i][j]
         local er  = torch.abs(gt - t)
         local err = torch.max(er)
         p(err)
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
                       e,cnt, maxerr,log.toc()))
end

function test.globalxyz2uv ()
   print("Testing globalxyz2uv") 
   local e        = 0
   local maxuverr = 0
   local maxpxerr = 0
   local cnt      = 0
   local gxyz     = test.data.global_positions
   local result   = test.data.result_globalxyz2uv
   log.tic()
   for i = 1,#photos do
      local photo = photos[i]
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local t     = torch.Tensor({photo:global_xyz_to_2d(gxyz[j])})
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
                       e,cnt, maxuverr,maxpxerr,log.toc()))
end


function test.localxy2globalray ()
   print("Testing localxy2globalray") 
   local e      = 0
   local maxerr = 0
   local cnt    = 0

   local gxyz   = test.data.global_positions
   log.tic()
   for i = 1,#photos do
      local photo = photos[i]
      for j = 1,gxyz:size(1) do
         cnt = cnt + 1
         local gtxyz  = gxyz[j]
         local t      = torch.Tensor({photo:global_xyz_to_2d(gtxyz)})
         local dir    = photo:local_xy_to_global_rot(t[3],t[4])
         local pt     = photo.position
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
                       e,cnt, maxerr,log.toc()))
end

-- tests 2globalray in context of photos
function test.localxy2globalray_photo ()
   print("Testing localxy2globalray_photos") 
   -- matterport textures go beyond 360 
   for pi = 1,#photos do
      local photo = photos[pi]
      local yerr = 0
      local xerr = 0
      local tot  = 0
      local lens = photo:get_lens().sensor
      local w  = lens.image_w
      local h  = lens.image_h
      
      local over = math.floor((w - lens.hfov * 1/360)*0.5 + 0.5)
      for y = 1,h,100 do 
         for x = over,w-over,100 do 
            local dir = photo:local_xy_to_global_rot(x, y)
            local pt  = photo.position
            local r = Ray.new(pt,dir)
            local vec = r(10)
            local u,v,nx,ny = photo:globalxyz2uv(vec)
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

function test.compute_dirs_offbyone()
   print("Testing compute directions off by one")
   local photo = photos[1]
   local scale = 16
   local invscale = 1/scale
   local lens = photo:get_lens().sensor
   local dirs  = photo:compute_dirs(scale)
   local err   = 0
   log.tic()
   local outh  = lens.image_h*invscale
   local outw  = lens.image_w*invscale
   for h = 1,outh do
      for w = 1,outw do
         local dir = photo:local_xy_to_global_rot((w-1)*scale,(h-1)*scale) 
         if torch.max(torch.abs(dir:narrow(1,1,3) - dirs[h][w])) > 1e-8 then
            err = err + 1
         end
      end
   end
   printf("-- %d/%d Errors in %2.2fs", err, outh*outw, log.toc()) 
end

function test.compute_dirs_deep()   
   print("Testing compute directions")
   
   for _,scale in pairs{16,8,4,2,1} do
      printf("Scale = %d",scale)
      local invscale = 1/scale
      for pi = 1,#photos do 
         local photo = photos[pi]
         local pt = photo.position
         local lens = photo:get_lens().sensor
         -- matterport textures go beyond 360 
         local over = torch.floor((lens.image_w - lens.hfov * 1/360)*0.5 + 0.5)
   
         local dirs  = photo:compute_dirs(scale)
         local xerr  = 0
         local yerr  = 0
         local tot   = 0
         log.tic()
         local outh  = lens.image_h*invscale
         local outw  = lens.image_w*invscale
         for h = 1,outh do
            for w = over[pi]+1,outw-over[pi] do
               local dir = dirs[h][w]
               local r = Ray.new(pt,dir)
               local v = r(1)
               local u,v,x,y = photo:globalxyz2uv(v)
               x = x*invscale
               y = y*invscale
               if (math.abs(h - y) > 1) then
                  printf("y: %d -> %f ", h, y) 
                  yerr = yerr + 1
               end
               if (math.abs(w- x) > 1) then
                  printf("x: %d -> %f ", w, x) 
                  xerr = xerr + 1
               end
               tot = tot + 1 
            end
         end
         printf(" - [%d] Errors: x:%d y:%d both: %d/%d", 
                pi, xerr, yerr, xerr + yerr, tot) 
      end
   end
end

function test.all()
   test.global2local()
   test.globalxyz2uv()
   test.localxy2globalray()
-- test.localxy2globalray_photo()
   test.compute_dirs_offbyone()
-- test.compute_dirs_deep()
end


return test
