require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'


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

poses  = loadcache(posefile,posecache,util.pose.loadtxtfile)
source = loadcache(sourcefile,sourcecache,util.obj.load,3)
target = loadcache(targetfile,targetcache,util.obj.load,4)


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
   local tmp = torch.Tensor(obj.nfaces)
   local found = false
   local tested = {}
   for fi = 1,obj.nfaces do
      local verts  = obj.face_verts[fi]
      local nverts = obj.nverts_per_face[fi]
      intersection, tstd = 
         util.geom.ray_face_intersection(pt,dir,
                                         obj.normals[fi],obj.d[fi],
                                         obj.face_verts[fi]:narrow(1,1,obj.nverts_per_face[fi]))
      if intersection then
         if (tstd < d) then 
            d = tstd
            found = true
         end
      end
   end
   return d
end


posedir = paths.dirname(posefile)
-- function compute_occlusions ()
-- for each pose.  create a D layer in the source mesh
scale    = 8
invscale = 1/scale
for pi = 1,#poses do
   sys.tic()
   outh = poses.h[pi]*invscale
   outw = poses.w[pi]*invscale
   ddata  = torch.zeros(outh,outw)
   ddatafile = posedir .. poses[pi]:gsub(".jpg","-dist.t7")
   dimagefile = posedir .. poses[pi]:gsub(".jpg","-dist.png")
   for h = 1,outh do -- poses.h[pi] do
      for w = 1,outw do -- poses.w[pi] do
         local pt,dir = util.pose.localxy2globalray(poses,pi,w*scale,h*scale)
         ddata[h][w]  = get_occlusions(pt,dir,target)
      end
   end
   print("Saving " .. ddatafile)
   torch.save(ddatafile,ddata)
   ddata:mul(1/ddata:max())
   print("Saving " .. dimagefile)
   image.save(dimagefile,ddata)
   print("Processed pose "..pi.." in "..sys.toc().." secs")
   image.display(ddata)
end