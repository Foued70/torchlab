require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'

ffi = require("ffi")
ffi.cdef[[ int printf(const char *fmt, ...); ]]
printf = ffi.C.printf

local geom = util.geom
-- top level filenames
targetfile = "/Users/marco/lofty/test/invincible-violet/retexture-tworoom.obj"
sourcedir  = "/Users/marco/lofty/models//invincible-violet-3396_a_00/"
sourcefile = sourcedir .. "scanner371_job129001.obj"
posefile   = sourcedir .. "scanner371_job129001_texture_info.txt"

-- FIXME make a cache class
cachedir = "cache/"

sys.execute("mkdir -p " .. cachedir)

posecache   = cachedir .. posefile:gsub("/","_")   .. ".t7"
sourcecache = cachedir .. sourcefile:gsub("/","_") .. ".t7"
targetcache = cachedir .. targetfile:gsub("/","_") .. ".t7"

function loadcache (objfile,cachefile,loader,args)
   -- Process or load the poses
   if (paths.filep(cachefile)) then
      sys.tic()
      object = torch.load(cachefile)
      print(string.format("Loaded object from %s in %2.2fs", posecache, sys.toc()))
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      print(string.format("Saving object to %s", posecache))
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
                                    face_verts,nverts_per_face,
                                    ...)
   local args = {...}
   local groundt = nil
   local fid = nil
   if (#args > 0) then
      groundt = args[1]
      if (#args > 1) then 
         fid = args[2]
      end
   end

   local timer = torch.Timer.new()

   -- output depth map
   local dmap  = torch.Tensor(dirs:size(1)):fill(-1)

   -- nfaces x ndirections
   distances = fast_ray_plane_intersection(pt,dirs,norms,d)
   local time0 = timer:time()
   print(string.format("Compute distances: %2.4fs",time0.real))

   -- angle between plane_norms and direction
   -- angles < 0 and at inf can be skipped
   local dists,index = torch.sort(distances,1)

   -- dists < 0.5 and > 1000m are not to be processed
   local valid = torch.gt(dists,0.5):cmul(torch.lt(dists,1e3))
   local vsum  = valid:sum(1)[1]
   local time1 = timer:time()
   print(string.format("Multiply and sort: %2.4fs",time1.real-time0.real))

   -- precompute the differences between consecutive face verts used
   -- for the line eq. when testing for intersection.
   local diff_verts = torch.Tensor(face_verts:size())
   local temp_verts = torch.Tensor(face_verts:size(2),face_verts:size(3))
   diff_verts[{{},1,{}}] = face_verts[{{},1,{}}] - face_verts[{{},face_verts:size(2),{}}]
   for i = 2,diff_verts:size(2) do 
      diff_verts[{{},i,{}}] = face_verts[{{},i,{}}] - face_verts[{{},i-1,{}}]
   end
   local nfaces = face_verts:size(1)
   -- precompute the indexes of the dominant dimensions for each face 
   local most_planar_normals = torch.IntTensor(nfaces,2)
   for i = 1,nfaces do
      local _,ds = torch.sort(torch.abs(norms[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end
   local time2 = timer:time()
   print(string.format("Compute planar norms: %2.4fs",time2.real-time1.real))


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
            
            local intersection = -(pt + dir * t)
            local nverts       = nverts_per_face[fidx]

            temp_verts:copy(face_verts[fidx])

            local ds           = most_planar_normals[fidx]
            -- move intersection to 0,0,0
            for vi = 1,nverts do
               temp_verts[vi]:add(intersection)
            end
            
            local time3_3 = timer:time()
            acc3_3 = acc3_3 + time3_3.real - time3_2_1.real
            
            -- count crossings along 'y' axis : b in slope intercept line equation
            local pvert = temp_verts[nverts]
            local count = 0
            for vi = 1,nverts do
               local cvert = temp_verts[vi]
               --  compute y axis crossing (b = y - mx)
               local run  =  cvert[ds[1]] - pvert[ds[1]]
               local b    = -math.huge
               local cpos = 1
               local ppos = 1
               if math.abs(run) < 1e-8 then
                  if (math.abs(cvert[ds[1]]) < 1e-8) then
                     count = count + 1
                  end
               else
                  b = cvert[ds[2]] - ((cvert[ds[2]] - pvert[ds[2]])/run) * cvert[ds[1]]
                  if (cvert[ds[1]] < 0) then cpos = -1 end
                  if (pvert[ds[1]] < 0) then ppos = -1 end
                  if (b >= 0) and ((cpos + ppos) == 0) then
                     count = count + 1
                  end
               end
               pvert = cvert
            end

            local time3_4 = timer:time()
            acc3_4 = acc3_4 + time3_4.real - time3_3.real

            if ((count > 0) and (count % 2)) then
               found = true
               dmap[i] = t -- this is the distance
            else
               j = j + 1
            end
         end
         visits = visits + visited
         local time3_5 = timer:time()
         acc3_5 = acc3_5 + time3_5.real - time3_2.real
      end
      if groundt and (torch.abs(dmap[i] - groundt[i]) > 1e-4) then
         print(valid:select(2,i))
         print(index:select(2,i))
         print(dists:select(2,i))
         print(distances:select(2,i))
         print(string.format("dir: %d face: %d <-> %d %2.4f <-> %2.4f = %2.4f visited: %d", 
                             i,index[j][i], fid[i], 
                             dmap[i], groundt[i], dmap[i] - groundt[i], 
                             visited))
      end
   end
   local time4 = timer:time()
   print(string.format(" - %2.4fs",acc3_1))
   print(string.format(" - scanning              : %2.4fs",acc3_2))
   print(string.format(" - centering face        : %2.4fs",acc3_3))
   print(string.format(" - count crossings       : %2.4fs",acc3_4))
   print(string.format(" - scan + intersect outer: %2.4fs",acc3_5))
   print(string.format("total face intersections : %2.4fs",time4.real-time2.real))
   print(string.format("    ***   TOTAL  ***     : %2.4fs",time4.real))
   print(string.format("visits per ray: %2.2f",visits/distances:size(2)))
   return dmap

end

-- FIXME Cache : not dependant on pose position. Can reuse between
-- multiple poses, by computing once for an image size and scale and
-- xdeg, ydeg and center.
--
-- FIXME Parallelize: ray packets, or simple increment if possible.  
function compute_dirs(p,i,scale)
   local imgw = p.w[i]
   local imgh = p.h[i]
   local dirs = torch.Tensor(math.ceil((imgw/scale))*math.ceil(imgh/scale),3)
   local cnt = 1
   for h = 1,imgh,scale do
      for w = 1,imgw,scale do
         local _,dir = util.pose.localxy2globalray(p,i,w-1,h-1)
         dirs[cnt]:copy(dir:narrow(1,1,3))
         cnt = cnt + 1
      end
   end
   return dirs
end

function compute_bbox()

end

function build_BVH_tree ()
   -- 1) all faces part of root node
   -- 2) pick dimension to split
   -- 3) intersect faces with 
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

function fast_get_occlusions(p,pi,obj,scale)
   local timer = torch.Timer.new()

   -- 1) set up local variables
   local dirs  = compute_dirs(p,pi,scale)
   local time0 = timer:time()
   print(string.format("Compute dirs: %2.4fs",time0.real))

   local pt    = p.xyz[pi] -- select pose xyz

   -- output depth map
   local dmap  = torch.Tensor(dirs:size(1)):fill(-1)

   local norms           = obj.normals
   local d               = obj.d
   local face_verts      = obj.face_verts
   local nverts_per_face = obj.nverts_per_face

   -- 2) compute distances to all planes for all directions
   -- local distances = fast_ray_plane_intersection(pt,dirs,obj.normals,obj.d)

   -- 3) compute face intersections
   local dmap = 
      fast_ray_face_intersection(pt,dirs,
                                 norms,d,
                                 face_verts,
                                 nverts_per_face)   
   
   return dmap
end

posedir = paths.dirname(posefile)
scale    = 4
invscale = 1/scale

function test_compute_dirs()
   print("Testing compute directions")
   local pi = 1
   local dirs = compute_dirs(poses,pi,scale)
   local err  = 0
   sys.tic()
   local outh = poses.h[pi]*invscale
   local outw = poses.w[pi]*invscale
   for h = 1,outh do
      for w = 1,outw do
         local pt,dir = util.pose.localxy2globalray(poses,pi,(w-1)*scale,(h-1)*scale)
         err = err + torch.sum(torch.abs(dir:narrow(1,1,3) - dirs[(h-1)*outw + w]))
      end
      print(string.format("-- Errors %2.2f in %2.2fs", err, sys.toc())) 
   end
end

-- test_compute_dirs()

function test_ray_plane_intersection()
   print("Testing ray plane intersections")
   local pi      = 1
   local dirs    = compute_dirs(poses,pi,scale)
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
               print(string.format("[%d][%d] %2.2f <-> %2.2f err: %2.2f", 
                                   fi,di,
                                   t,distances[fi][di],err))
            end
         end
      end
   end
   print(string.format("-- Errors: %d in %2.2fs", nerr, sys.toc()))
end
-- test_ray_plane_intersection()

function test_ray_face_intersection ()
   print("Testing ray face intersections")
   local pi    = 1
   local dirs  = compute_dirs(poses,pi,scale)
   local obj   = target
   local err   = 0
   local norms = obj.normals
   local d     = obj.d
   local pt    = poses.xyz[pi] 
   local face_verts = obj.face_verts
   local nverts_per_face  = obj.nverts_per_face
   -- dirs = dirs:narrow(1,57570,1)
   slow_ds = torch.Tensor(dirs:size(1))
   slow_fids = torch.IntTensor(dirs:size(1))
   sys.tic()
 
   for di = 1,dirs:size(1) do
      slow_ds[di],slow_fids[di] = get_occlusions(pt,dirs[di],obj)
   end

   fast_ds = fast_ray_face_intersection(pt,dirs,
                                        norms,d,
                                        face_verts,nverts_per_face,
                                        slow_ds,slow_fids)   
   local errs = torch.abs(fast_ds - slow_ds)
   local err  = torch.max(errs)
   print(string.format("-- %d/%d Errors: ",torch.sum(torch.gt(errs,1e-8)),
                       errs:size(1)))
end

-- test_ray_face_intersection()

for pi = 1,1 do -- #poses do
   sys.tic()
   dmap = fast_get_occlusions(poses,pi,target,scale)
   print(string.format("Computed fast occlusions in %2.4fs", sys.toc()))

   sys.tic()
   outh       = poses.h[pi]*invscale
   outw       = poses.w[pi]*invscale
   ddata      = torch.zeros(outh,outw)
   ddatafile  = posedir .. poses[pi]:gsub(".jpg","-dist.t7")
   dimagefile = posedir .. poses[pi]:gsub(".jpg","-dist.png")
   for h = 1,outh do
      for w = 1,outw do
         local pt,dir = util.pose.localxy2globalray(poses,pi,(w-1)*scale,(h-1)*scale)
         ddata[h][w]  = get_occlusions(pt,dir,target)
      end
   end
   print("Processed pose "..pi.." in "..sys.toc().." secs")
   dmap:resize(ddata:size())
   image.display{image={ddata,dmap,torch.abs(ddata-dmap)}, nrow=1}
end
