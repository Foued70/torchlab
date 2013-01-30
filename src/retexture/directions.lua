require 'sys'

local util = require 'util'
local pose = util.pose
-- FIXME make this part of pose object
--
-- + FIXME improve speed : creation with a simple increment if possible. (SLERP)

function compute_dirs(p,i,scale)

   local imgw = p.w[i]
   local imgh = p.h[i]

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   -- dirs are 2D x 3
   local dirs = torch.Tensor(outh,outw,3)
   local cnt = 1
   local inh = 0
   local inw = 0
   for h = 1,outh do
      for w = 1,outw do
         local _,dir = pose.localxy2globalray(p,i,inw,inh)
         dirs[h][w]:copy(dir:narrow(1,1,3))
         cnt = cnt + 1
         inw = inw + scale
      end
      inh = inh + scale
      inw = 0
   end
   return dirs
end

-- 
-- Caching
-- 
-- is not dependant on pose position or other pose dependent
-- information other than width, height, scale and center, and thus can be
-- reused between multiple poses, by computing once for an image size
-- and scale and xdeg, ydeg and center.
function load_dirs(p,i,scale,ps)

   local imgw = p.w[i]
   local imgh = p.h[i]

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = p.cntrx[i]
   local cntry = p.cntry[i]

   local dirscache   = cachedir .. 
      "orig_"..imgw.."x"..imgh.."_-_"..
      "scaled_"..outw.."x"..outh.."_-_"..
      "center_"..cntrx.."x"..cntry

   if ps then 
      dirscache = dirscache .."_-_grid_".. ps
   end
   dirscache = dirscache ..".t7"

   local dirs = nil
   if paths.filep(dirscache) then
      sys.tic()
      dirs = torch.load(dirscache)
      printf("Loaded dirs from %s in %2.2fs", posecache, sys.toc())
   else
      if ps then 
         dirs = grid_contiguous(compute_dirs(p,i,scale),ps,ps)
      else
         dirs = compute_dirs(p,i,scale)
      end
      torch.save(dirscache,dirs)
      printf("Saving dirs to %s", dirscache)
   end
   return dirs
end

-- is unique for each scale and pose.
function load_depth(cachedir,p,i,scale,ps)

   local imgw = p.w[i]
   local imgh = p.h[i]

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = p.cntrx[i]
   local cntry = p.cntry[i]

   local depthcache   = cachedir .. 
      "orig_"..imgw.."x"..imgh.."_-_"..
      "scaled_"..outw.."x"..outh.."_-_"..
      "center_"..cntrx.."x"..cntry

   dirscache = dirscache ..".t7"

   local depthmap = nil
   if paths.filep(dirscache) then
      sys.tic()
      depthmap = torch.load(dirscache)
      printf("Loaded depths from %s in %2.2fs", posecache, sys.toc())
      return depthmap
   end
   return nil
end


-- Parallelization: 
-- 
-- + use ray packets 
-- 
-- + produce and cache contiguous packets on a grid.  Like unfold but
-- produces contiguous chunks which would replicate data in overlaps.
-- Start with a less general version which takes h,w,dims matrix as
-- input and outputs r,c,s1*s2,dims as output. (Contiguous unfold
-- useful for bilateral filtering also).
function grid_contiguous (m,s1,s2)
   local uf = m:unfold(1,s1,s1):unfold(2,s2,s2)
   local out = torch.Tensor(uf:size(1),uf:size(2),uf:size(4)*uf:size(5),uf:size(3))
   for r = 1,uf:size(1) do
      for c = 1,uf:size(2) do
         out[r][c]:copy(uf[r][c])
      end
   end
   return out
end

function test_grid_contiguous()
   print("Testing grid contiguous")
   local ww  = 4
   local m   = torch.randn(32,32,3)
   local ufm = grid_contiguous(m,ww,ww)
   local err = 0
   local errc = 0
   for r = 1,ufm:size(1) do 
      for c = 1,ufm:size(2) do 
         local ow = m:narrow(1,1+(r-1)*ww,ww):narrow(2,1+(c-1)*ww,ww)            
         if 1e-8 < torch.sum(ufm[r][c] - ow) then 
            err = err + 1
         end
         if not ufm[r][c]:isContiguous() then
            errc = errc + 1
         end
      end
   end
   local ntests = ufm:size(1)*ufm:size(2)
   printf("-- %d/%d Errors/ %d/%d not contiguous ",err,ntests,errc,ntests)
end

function test_compute_dirs_offbyone(poses,pi,scale)
   print("Testing compute directions off by one")
   if not pi then pi = 1 end
   if not scale then scale = 4 end
   local invscale = 1/scale

   local dirs  = compute_dirs(poses,pi,scale)
   local err   = 0
   sys.tic()
   local outh  = poses.h[pi]*invscale
   local outw  = poses.w[pi]*invscale
   for h = 1,outh do
      for w = 1,outw do
         local pt,dir = pose.localxy2globalray(poses,pi,(w-1)*scale,(h-1)*scale) 
         if torch.max(torch.abs(dir:narrow(1,1,3) - dirs[h][w])) > 1e-8 then
            err = err + 1
         end
      end
   end
   printf("-- %d/%d Errors in %2.2fs", err, outh*outw, sys.toc()) 
end

function test_compute_dirs_deep(poses)
   require 'ray'
   print("Testing compute directions")
   for _,scale in pairs{16,8,4,2,1} do
      printf("Scale = %d",scale)
      local invscale = 1/scale
      for pi = 1,poses.nposes do 
         local pt = porig.xyz[pi]
         -- matterport textures go beyond 360 
         local over = torch.floor((porig.w - 360 / porig.px[1][1])*invscale*0.5 + 0.5)

         local dirs  = compute_dirs(poses,pi,scale)
         local xerr  = 0
         local yerr  = 0
         local tot   = 0
         sys.tic()
         local outh  = poses.h[pi]*invscale
         local outw  = poses.w[pi]*invscale
         for h = 1,outh do
            for w = over[pi]+1,outw-over[pi] do
               local dir = dirs[h][w]
               local r = Ray(pt,dir)
               local v = r(1)
               local u,v,x,y = pose.globalxyz2uv(poses,pi,v)
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

function run_all_tests()
   test_grid_contiguous()
   test_compute_dirs_off_by_one()
end