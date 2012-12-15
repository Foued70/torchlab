require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'


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

poses  = loadcache(posefile,posecache,util.pose.load)
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

-- pt (point) is the starting point of the ray
-- dir (direction) is the (normalized) direction it travels
-- plane is the a,b,c,d (normal(a,b,c),offset(d)) eq. for the plane
function ray_plane_intersection(pt,dir,plane)
   local plane_norm = plane:narrow(1,1,3)
   local plane_d    = plane[4]
   local angle = torch.dot(plane:narrow(1,1,3),ray)
   if (math.abs(angle) < 1e-8) then
      return nil
   end
   local t = - (torch.dot(plane_norm,pt) + plane_d)/angle
   if (t < 0) then 
      return nil
   end
   return pt + dir * t
end

# is the intersection inside the face?
# http://www.siggraph.org/education/materials/HyperGraph/raytrace/raypolygon_intersection.htm

function ray_face_intersection(pt,dir,plane,face_verts)
   local plane_norm = plane:narrow(1,1,3)
   local plane_d    = plane[4]
   local intersection = ray_plane_intersection(pt,dir,plane)
   if intersection then
      local v,i   = plane_norm:sort()
      local ri    = torch.Tensor(2)
      ri[1] = intersection[i[1]]
      ri[2] = intersection[i[1]]
      local verts = torch.Tensor(face_verts:size(1),2)
      for j in 1,face_verts:size(1) do
         verts[
      
   end
   return nil
end
-- compute occlusions

function compute_occlusions