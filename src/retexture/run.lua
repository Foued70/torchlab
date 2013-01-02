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
   for fi = 1,obj.nfaces do
      local verts  = obj.face_verts[fi]
      local nverts = obj.nverts_per_face[fi]
      local intersection, tstd =
         util.geom.ray_face_intersection(pt,dir,
                                         obj.normals[fi],obj.d[fi],
                                         obj.face_verts[fi]:narrow(1,1,obj.nverts_per_face[fi]))
      if intersection then
         if (tstd < d) then
            d = tstd
         end
      end
   end
   return d
end


function fast_ray_face_intersection(pt,dir,normals,d,face_verts)
   -- angle between plane_norms and direction

end

-- can cache in the pose and parallelize
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

function fast_get_occlusions(p,i,obj,scale)
   local timer = torch.Timer.new()
   -- FIXME slice dirs to limit the number of angles computed at a
   -- time
   local dirs   = compute_dirs(p,i,scale)
   local time0 = timer:time()
   print(string.format("Compute dirs: %2.4fs",time0.real))
   local pt = p.xyz[i]
   local dmap = torch.Tensor(dirs:size(1)):fill(-1)
   -- precompute the differences between consecutive face verts used
   -- for the line eq. when testing for intersection.
   local face_verts = obj.face_verts
   local diff_verts = torch.Tensor(face_verts:size())
   local temp_verts = torch.Tensor(face_verts:size(2),face_verts:size(3))
   diff_verts[{{},1,{}}] = face_verts[{{},1,{}}] - face_verts[{{},face_verts:size(2),{}}]
   for i = 2,diff_verts:size(2) do 
      diff_verts[{{},i,{}}] = face_verts[{{},i,{}}] - face_verts[{{},i-1,{}}]
   end
   local most_planar_normals = torch.IntTensor(obj.nfaces,2)
   for i = 1,obj.nfaces do
      local _,ds = torch.sort(torch.abs(obj.normals[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end
   local time1 = timer:time()
   print(string.format("Compute planar norms: %2.4fs",time1.real-time0.real))
   -- pow(-1) is robust to 0s. will place inf in cell
   -- angles is nfaces * ndirs
   local angles = torch.mm(obj.normals,dirs:t()):pow(-1)
   local time2  = timer:time()
   print(string.format("Compute angles: %2.4fs",time2.real-time1.real))
   -- ts is vector of nfaces
   local ts     = - (torch.mv(obj.normals,pt) + obj.d)
   local time3  = timer:time()
   print(string.format("Compute distances: %2.4fs",time3.real-time2.real))
   local acc3_1 = 0
   local acc3_2 = 0
   local acc3_3 = 0
   local acc3_4 = 0
   local acc3_5 = 0

   local time3_0 = timer:time()

   -- iterate over directions here FIXME move to ffi
   for i = 1,angles:size(2) do
      angles:select(2,1):cmul(ts)
   end

   -- angles < 0 and at inf can be skipped
   local dists,index = torch.sort(angles,1)

   -- dists < 0.5 and > 1000m are not to be processed
   local valid = torch.gt(dists,0.5):cmul(torch.lt(dists,1e3))
   local vsum  = valid:sum(1)[1]
   local time3_1 = timer:time()
   print(string.format("Multiply and sort: %2.4fs",time3_1.real-time3_0.real))
   local visits = 0
   -- loop over directions
   for i = 1,angles:size(2) do
      if vsum[i] > 0 then
         time3_1 = timer:time()
         local j = 1
         local v = valid[j][i]
         local dir = dirs[i]
         -- FIXME speed up with bsearch
         while (v == 0) and (j < (obj.nfaces + 1)) do
            j = j + 1
            v = valid[j][i]
         end
         local time3_2 = timer:time()
         acc3_2 = acc3_2 + time3_2.real - time3_1.real

         local found = false
         local visited = 0
         while (not found) and (v == 1) and (j < (obj.nfaces + 1)) do
            local time3_2_1 = timer:time()
            visited = visited + 1
            v = valid[j][i]
            t = dists[j][i]
            local intersection = -(pt + dir * t)
            local fidx         = index[j][i]
            local nverts       = obj.nverts_per_face[fidx]

            temp_verts:copy(diff_verts[fidx])

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
   end
   local time4 = timer:time()
   print(string.format(" - %2.4fs",acc3_1))
   print(string.format(" - %2.4fs",acc3_2))
   print(string.format(" - %2.4fs",acc3_3))
   print(string.format(" - %2.4fs",acc3_4))
   print(string.format(" - %2.4fs",acc3_5))
   print(string.format("total face intersections: %2.4fs",time4.real-time3.real))
   print(string.format("  *** TOTAL ***   : %2.4fs",time4.real))
   print(string.format("visits per ray: %2.2f",visits/angles:size(2)))
   return dmap
end


posedir = paths.dirname(posefile)
scale    = 4
invscale = 1/scale
for pi = 1,1 do -- #poses do
   sys.tic()
   dmap = fast_get_occlusions(poses,pi,target,scale)
   print(string.format("Computed fast occlusions in %2.4fs", sys.toc()))

   sys.tic()
   outh = poses.h[pi]*invscale
   outw = poses.w[pi]*invscale
   ddata  = torch.zeros(outh,outw)
   ddatafile = posedir .. poses[pi]:gsub(".jpg","-dist.t7")
   dimagefile = posedir .. poses[pi]:gsub(".jpg","-dist.png")
   for h = 1,outh do
      for w = 1,outw do
         local pt,dir = util.pose.localxy2globalray(poses,pi,(w-1)*scale,(h-1)*scale)
         ddata[h][w]  = get_occlusions(pt,dir,target)
      end
   end
   print("Processed pose "..pi.." in "..sys.toc().." secs")
   image.display(ddata)
end