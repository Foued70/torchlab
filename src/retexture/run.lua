require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'

-- ffi = require("ffi")
-- ffi.cdef[[ int printf(const char *fmt, ...); ]]
-- printf = ffi.C.printf

function printf(...)
   print(string.format(...))
end

local geom = util.geom
-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
cmd:text('Options')
cmd:option('-targetfile',
           "/Users/marco/lofty/test/invincible-violet/retexture-tworoom.obj",
           'target obj with new geometry')
cmd:option('-sourcefile',
           "/Users/marco/lofty/models//invincible-violet-3396_a_00/scanner371_job129001.obj",
           'source obj')
cmd:option('-posefile',
           "/Users/marco/lofty/models//invincible-violet-3396_a_00/scanner371_job129001_texture_info.txt",
           'pose info file in same directory as the texture images')
cmd:option('-scale',4,'scale at which to process 4 = 1/4 resolution')
cmd:option('-packetsize',32,'window size for ray packets (32x32)')
cmd:text()
 
-- parse input params
params = cmd:parse(arg)

targetfile = params.targetfile
sourcefile = params.sourcefile
posefile   = params.posefile
scale      = params.scale
packetsize = params.packetsize

cachedir = "cache/"

sys.execute("mkdir -p " .. cachedir)

posecache   = cachedir .. posefile:gsub("/","_")   .. ".t7"
sourcecache = cachedir .. sourcefile:gsub("/","_") .. ".t7"
targetcache = cachedir .. targetfile:gsub("/","_") .. ".t7"
posedir = paths.dirname(posefile)

function loadcache (objfile,cachefile,loader,args)
   local object = nil
   -- Process or load the poses
   if (paths.filep(cachefile)) then
      sys.tic()
      object = torch.load(cachefile)
      printf("Loaded %s from %s in %2.2fs", objfile, cachefile, sys.toc())
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      printf("Saving %s to %s", objfile, cachefile)
   end
   return object
end

if not poses then
   poses  = loadcache(posefile,posecache,util.pose.loadtxtfile)
end
if not source then
   source = loadcache(sourcefile,sourcecache,util.obj.load,3)
end
if not target then
   target = loadcache(targetfile,targetcache,util.obj.load,4)
end

ppm        = 100 -- pixels per meter
nposes     = 5   -- max number of poses to consider per texture
mindist    = 0.7 -- min distance to scanner
mindistsqr = mindist*mindist
ideal      = 1.5 -- meters for fade
maxdist    = 50
idealdist  = 2   -- prefered distance in meters
vertbuffer = 0   -- how close in pixels to edge of texture do we accept

pi  = math.pi
pi2 = pi/2

-- simple linear fadein/out eqn.
idealdist   =  ideal + mindist

-- line 1 from (mindist,0) -> (mindist+ideal,1)
fin_a   =  1/ideal
fin_b   =  1 - fin_a * idealdist

-- line 2 from (mindist+ideal,1) -> (maxdist,0)
fout_a  = -1/(maxdist - idealdist)
fout_b  =  1 - fout_a * idealdist

-- do we blend or take max
doMax = True

function get_occlusions(pt,dir,obj)
   local d = math.huge
   dir = dir:narrow(1,1,3)
   fid = 0
   for fi = 1,obj.nfaces do
      local verts  = obj.face_verts[fi]
      local nverts = obj.nverts_per_face[fi]
      local intersection, tstd =
         util.geom.ray_face_intersection(
         pt,dir,
         obj.normals[fi],obj.d[fi],
         obj.face_verts[fi]:narrow(1,1,obj.nverts_per_face[fi]))
      if intersection then
         if (tstd < d) then
            d = tstd
            fid = fi
         end
      end
   end
   return d,fid
end



function fast_ray_face_intersection(pt,dirs,
                                    norms,d,
                                    face_verts,most_planar_normals,
                                    nverts_per_face,
                                    ...)
   -- optional debugging arguments
   local args = {...}
   local groundt = nil
   local fid = nil
   if (#args > 0) then
      groundt = args[1]
      if (#args > 1) then 
         fid = args[2]
      end
   end

   local debug = false

   local timer = torch.Timer.new()

   -- output depth map (FIXME should pass in if we want to fill and avoid the copy)
   local dmap  = torch.Tensor(dirs:size(1)):fill(0)

   -- nfaces x ndirections
   distances = fast_ray_plane_intersection(pt,dirs,norms,d)
   local time0 = timer:time()
   printf("Compute distances: %2.4fs",time0.real)

   -- angle between plane_norms and direction
   -- angles < 0 and at inf can be skipped
   local dists,index = torch.sort(distances,1)

   -- dists < 0 are behind camera and are not to be processed
   local valid = torch.gt(dists,0) 
   local vsum  = valid:sum(1)[1]
   local time1 = timer:time()
   printf("Multiply and sort: %2.4fs",time1.real-time0.real)

   local nfaces = face_verts:size(1)
   local temp_verts = torch.Tensor(face_verts:size(2),face_verts:size(3))
   
   local time2 = timer:time()
   printf("Compute planar norms: %2.4fs",time2.real-time1.real)


   local time_3_1 = timer:time()
   local acc3_1 = 0
   local acc3_2 = 0
   local acc3_3 = 0
   local acc3_4 = 0
   local acc3_5 = 0

   local visits = 0

   -- loop over directions in the distances. 
   -- Each direction has a set of distances to each face
   for i = 1,distances:size(2) do
      local j = 1 
      local found = false
      local visited = 0
      if vsum[i] > 0 then
         time3_1 = timer:time()
         local v   = valid[j][i]
         local dir = dirs[i]
         -- FIXME speed up with bsearch
         while (v == 0) and (j < (nfaces + 1)) do
            j = j + 1
            v = valid[j][i]
         end
         local time3_2 = timer:time()
         acc3_2 = acc3_2 + time3_2.real - time3_1.real

         while (not found) and (v == 1) and (j < (nfaces + 1)) do
            local time3_2_1 = timer:time()
            visited    = visited + 1
            v          = valid[j][i]
            local fidx = index[j][i]
            local    t = dists[j][i]
            
            -- FIXME do a batch of directions at a time
            local intersection = pt + dir * t
            local nverts       = nverts_per_face[fidx]
            
            local ds           = most_planar_normals[fidx]

            local time3_3 = timer:time()
            acc3_3 = acc3_3 + time3_3.real - time3_2_1.real

            -- faster no copies less substraction
            found = 
               point_in_polygon(intersection,
                                face_verts[fidx]:narrow(1,1,nverts),
                                ds)
            local time3_4 = timer:time()
            acc3_4 = acc3_4 + time3_4.real - time3_3.real
            if found then 
               dmap[i] = t
            else
               j = j + 1 
            end
         end -- while
         local time3_5 = timer:time()
         acc3_5 = acc3_5 + time3_5.real - time3_2.real
      end -- if valid
      visits = visits + visited
            
      if groundt and (torch.abs(dmap[i] - groundt[i]) > 1e-4) then
         print(valid:select(2,i))
         print(index:select(2,i))
         print(dists:select(2,i))
         print(distances:select(2,i))
         printf("dir: %d face: %d <-> %d %2.4f <-> %2.4f = %2.4f visited: %d", 
                             i,index[j][i], fid[i], 
                             dmap[i], groundt[i], dmap[i] - groundt[i], 
                             visited)
      end
   end -- for each direction 
   local time4 = timer:time()
   printf(" - scanning              : %2.4fs",acc3_2)
   printf(" - setup                 : %2.4fs",acc3_3)
   printf(" - point in polygon(new) : %2.4fs",acc3_4)
   printf(" - scan + intersect outer: %2.4fs",acc3_5)
   printf("total face intersections : %2.4fs",time4.real-time2.real)
   printf("    ***   TOTAL  ***     : %2.4fs",time4.real)
   printf("visits per ray: %2.2f",visits/distances:size(2))
   return dmap

end

-- FIXME part of pose object
-- 
-- FIXME Cache : not dependant on pose position. Can reuse between
-- multiple poses, by computing once for an image size and scale and
-- xdeg, ydeg and center.
--
-- FIXME Parallelization: 
-- 
-- + use ray packets 
-- 
-- + produce and cache contiguous packets on a grid. (Contiguous
-- unfold useful for bilateral filtering also).
-- 
-- + speed : creation with a simple increment if possible. (SLERP)

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
         local _,dir = util.pose.localxy2globalray(p,i,inw,inh)
         dirs[h][w]:copy(dir:narrow(1,1,3))
         cnt = cnt + 1
         inw = inw + scale
      end
      inh = inh + scale
      inw = 0
   end
   return dirs
end

function load_dirs(p,i,scale,ps)

   local imgw = p.w[i]
   local imgh = p.h[i]

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = p.cntrx[i]
   local cntry = p.cntry[i]

   local dirscache   = cachedir .. 
      imgw.."x"..imgh.."_-_"..
      outw.."x"..outh.."_-_"..
      cntrx.."x"..cntry

   if ps then 
      dirscache = dirscache .."_-_".. ps
   end
   dirscache = dirscache ..".t7"

   local dirs = nil
   if paths.filep(dirscache) then
      sys.tic()
      dirs = torch.load(dirscache)
      printf("Loaded dirs from %s in %2.2fs", objfile, posecache, sys.toc())
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

      

-- pt (point) is the intersection between the ray and the plane of the polygon
-- verts are the vertices of the polygon
-- dims are the precomputed dominant dimensions in which we flatten the polygon to compute
function point_in_polygon(pt,verts,dims)
   -- dims 1 == X axis
   -- dims 2 == Y axis
   local x         = dims[1]
   local y         = dims[2]
   local nverts    = verts:size(1)
   local inside    = false
   local p1        = verts[nverts] 
   local yflag1    = (p1[y] >= pt[y])
   for vi = 1,nverts do
      local p2     = verts[vi]
      local yflag2 = (p2[y] >= pt[y])
      -- do we have a crossing ?
      if (yflag1 ~= yflag2) then
         -- no division test of positive intersection with xaxis
         if (yflag2 == 
             ((p2[y] - pt[y])*(p1[x] - p2[x]) >= (p2[x] - pt[x])*(p1[y] - p2[y]))) then
            inside = not inside
         end
      end
      yflag1 = yflag2
      p1 = p2
   end
   return inside
end

-- takes a bunch of rays (pt, dirs) and a bunch of planes (normals,d)
-- and computes the distance to all planes for each direction using
-- matrix operations.
function fast_ray_plane_intersection(pt,dirs,normals,d)
   -- # compute distance along ray (pt, in all directions dir) to all face planes
   -- pow(-1) is robust to 0s. will place inf in cell
   -- distances is nfaces * ndirs
   -- matrix version of : angles = 1/(normal dot dir)
   local distances = torch.mm(normals,dirs:t()):pow(-1)
   
   -- ts is vector of nfaces
   -- normals dot xyz + plane_d (t's are distance along the ray where
   -- the plane intersection happens

   local ts     = - (torch.mv(normals,pt) + d)

   -- iterate over directions here FIXME move to ffi
   for i = 1,distances:size(2) do
      distances:select(2,i):cmul(ts)
   end

   return distances
end

function fast_get_occlusions(p,pi,obj,scale,ps)
   local timer = torch.Timer.new()

   -- 1) set up local variables
   local dirs = load_dirs(p,pi,scale,ps)
   local time0 = timer:time()

   printf("Compute dirs: %2.4fs",time0.real)

   local pt    = p.xyz[pi] -- select pose xyz

   -- output depth map
   local dmap  = torch.Tensor(dirs:size(1)*ps,dirs:size(2)*ps):fill(0)

   -- FIXME pass obj to face intersection
   local norms           = obj.normals
   local d               = obj.d
   local face_verts      = obj.face_verts
   local nverts_per_face = obj.nverts_per_face

   -- 2) precompute dominant dimensions for all edges of
   -- all faces precompute the differences between consecutive face
   -- verts used for the line eq. when testing for intersection. FIXME
   -- move this to a face object. FIXME move to obj.

   -- precompute the indexes of the dominant dimensions for each face 
   local nfaces = face_verts:size(1)
   local most_planar_normals = torch.IntTensor(nfaces,2)
   for i = 1,nfaces do
      local _,ds = torch.sort(torch.abs(norms[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end

   local orow = 1
   local ocol = 1
   local ray_packet = nil
   for r = 1,dirs:size(1) do 
      for c = 1,dirs:size(2) do 
         printf("[%d][%d] of [%d][%d]",orow,ocol,dmap:size(1),dmap:size(2))
         ray_packet = dmap:narrow(1,orow,ps):narrow(2,ocol,ps)
         ray_packet:copy( 
            fast_ray_face_intersection(pt,dirs[r][c],
                                       norms,d,
                                       face_verts,most_planar_normals,
                                       nverts_per_face))
         ocol = ocol + ps
      end
      orow = orow + ps
      ocol = 1
   end
   
   return dmap
end


-- like unfold but produces contiguous chunks.
-- start with a less general version which takes h,w,dims matrix as input and outputs
-- r,c,s1*s2,dims as output
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
   print("Testing unfold contiguous")
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


function test_compute_dirs()
   print("Testing compute directions")
   local pi    = 1
   local scale = 4
   local dirs  = compute_dirs(poses,pi,scale)
   local err   = 0
   sys.tic()
   local outh  = poses.h[pi]/scale
   local outw  = poses.w[pi]/scale
   for h = 1,outh do
      for w = 1,outw do
         local pt,dir = util.pose.localxy2globalray(poses,pi,(w-1)*scale,(h-1)*scale)
         if torch.max(torch.abs(dir:narrow(1,1,3) - dirs[h][w])) > 1e-8 then
            err = err + 1
         end
      end
   end
   printf("-- %d/%d Errors in %2.2fs", err, outh*outw, sys.toc())
end


function test_ray_plane_intersection()
   print("Testing ray plane intersections")
   local pi      = 1
   local dirs    = compute_dirs(poses,pi,scale)
   dirs:resize(dirs:size(1)*dirs:size(2),3)

   local obj     = target
   local err     = 0
   local norms   = obj.normals
   local d       = obj.d

   sys.tic()

   local pt      = poses.xyz[pi] 

   local dots    = torch.mm(norms,dirs:t())
   local invdots = torch.pow(dots,-1)
   local ts      = - ( torch.mv(norms,pt) + d)
   
   local dists   = torch.Tensor(dots:size()):copy(invdots)

   -- iterate over directions 
   for i = 1,dists:size(2) do
      dists:select(2,i):cmul(ts) 
   end

   -- nfaces x ndirections
   local distances = fast_ray_plane_intersection(pt,dirs,norms,d)
   local nerr      = 0

   for fi = 1,norms:size(1) do
      for di = 1,dirs:size(1) do
         local dot = torch.dot(norms[fi],dirs[di])
         if torch.abs(dot - dots[fi][di]) > 1e-8 then
            print("dot product error")
         end
         if torch.abs(1/dot - invdots[fi][di]) > 1e-8 then
            print("inv dot error")
         end
         local t = -( torch.dot(norms[fi],pt) + d[fi])
         if torch.abs(t - ts[fi]) > 1e-8 then
            print("initial t error")
         end
         if torch.abs(t/dot - dists[fi][di]) > 1e-8 then
            print("final error")
         end
         i,t = util.geom.ray_plane_intersection(pt,dirs[di],norms[fi],d[fi])
         if i then
            local err = torch.abs(t - distances[fi][di])
            if err > 1e-8 then
               nerr = nerr + 1
               printf("[%d][%d] %2.2f <-> %2.2f err: %2.2f", 
                                   fi,di,
                                   t,distances[fi][di],err)
            end
         end
      end
   end
   printf("-- Errors: %d in %2.2fs", nerr, sys.toc())
end

function test_ray_face_intersection ()
   print("Testing ray face intersections")
   local pi    = 1
   local dirs  = compute_dirs(poses,pi,scale)
   dirs = dirs:resize(dirs:size(1)*dirs:size(2),3)
   local obj   = target
   local err   = 0
   local norms = obj.normals
   local d     = obj.d
   local pt    = poses.xyz[pi] 
   local face_verts = obj.face_verts
   local nverts_per_face  = obj.nverts_per_face

   -- precompute the indexes of the dominant dimensions for each face 
   local nfaces = face_verts:size(1)
   local most_planar_normals = torch.IntTensor(nfaces,2)
   for i = 1,nfaces do
      local _,ds = torch.sort(torch.abs(norms[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end

   dirs      = dirs:narrow(1,57570,10)
   slow_ds   = torch.Tensor(dirs:size(1))
   slow_fids = torch.IntTensor(dirs:size(1))
   sys.tic()
 
   for di = 1,dirs:size(1) do
      slow_ds[di],slow_fids[di] = get_occlusions(pt,dirs[di],obj)
   end

   fast_ds = fast_ray_face_intersection(pt,dirs,
                                        norms,d,
                                        face_verts,
                                        most_planar_normals,
                                        nverts_per_face,
                                        slow_ds,slow_fids)   
   local errs = torch.abs(fast_ds - slow_ds)
   local err  = torch.max(errs)
   printf("-- %d/%d Errors",torch.sum(torch.gt(errs,1e-8)),
                       errs:size(1))
end

function run_all_tests()
   test_grid_contiguous()
   test_compute_dirs()
   -- test_ray_plane_intersection()
   test_ray_face_intersection()
end
run_all_tests()

-- for pi = 4,4 do -- #poses do
--    sys.tic()
--    dmap = fast_get_occlusions(poses,pi,target,scale,packetsize)
--    printf("Computed fast occlusions in %2.4fs", sys.toc())

--    image.display(dmap)
-- end
