require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'


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
      printf("Loaded %s from %s in %2.2fs\n", objfile, cachefile, sys.toc())
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      printf("Saving %s to %s\n", objfile, cachefile)
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

-- more args FIXME become arguements
ppm        = 500 -- pixels per meter
mpp        = 1/ppm -- meters per pixel
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

function draw_wireframe (p,i,obj)
   local pimage = p.images[i]
   local psize = pimage:size()
   psize[1] = 4
   local wimage = torch.Tensor(psize):fill(0)
   local face_verts = obj.face_verts
   for fi = 1,face_verts:size(1) do
      local f = face_verts[fi]
      local pvert = f[f:size(1)]
      for vi = 1,f:size(1) do
         local cvert = f[vi]
         local dir = cvert - pvert
         local len = torch.norm(dir)
         if (len > 1e-8) then
            dir = dir/len
            step = dir * mpp
            -- printf("step: %f,%f,%f\n",step[1],step[2],step[3])
            for s = 0,len,mpp do 
               -- draw verts first
               local u,v,x,y = util.pose.globalxyz2uv(p,i,pvert)
               -- printf("u: %f v: %f x: %f y %f\n", u, v, x, y)
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

for pi = 1,poses.nposes do 
   wimage = draw_wireframe(poses,pi,target)
   image.display(wimage)
   -- save
   wimagename = poses[pi]:gsub(".jpg","_wireframe.png")
   printf("Saving: %s\n", wimagename)
   image.save(wimagename,wimage)
end

-- TODO 
-- 1) redo retexture (no optimisations just straight up) and write obj file.
--   a) debug: find UV in pose for a texture (test pose globalxyz2uv function)
--   b) draw wireframe on pose.

--   -- idea: projecting whole wireframe onto pose would help with
--      alignment of all faces at once (Reduce human alignment
--      tweaks).

-- Main Functions:

-- + ----------------
-- get_closest_poses() 

-- for a given face select the best poses we will use to color it.

--  a) find poses which are closest (to each vertex, or face center)
--     and on the correct side of the face normal
--  
--  b) reject poses where the camera is too close.  Can reject just on
--     the face center as we prefer whole face to be colored by a
--     single pose.

--  c) make sure that at least one vertex of face falls within the
--     poses view. (not sure we need this)

-- + ----------------
-- face_to_texture_transform_and_dimensions() 

-- find rotatation and translation for a virutal camera centered on
-- the face and the dimensions of a texture map

--  a) find translation (point closest to origin of longest edge)

--  b) find dimensions of the texture to be created

-- + ----------------
-- compute_alpha(dir,d,norm)
        
--         # smaller angle better (normalize w/ pi/2 as that is the biggest angle)
--         a = 1 - (np.arccos(np.dot(-dir,norm))/self.pi2)
--         # print(" - d: %f a: %f" %(d,a))
--         if (d < self.mindist) or (d > self.maxdist) or (np.abs(a) > self.pi2):
--             return 0
--         elif(d < self.idealdist):
--             return a*(d*self.fin_a + self.fin_b)
--         else: 
--             return a*(d*self.fout_a + self.fout_b)

-- + ----------------
-- remap_uvs()
--         for i,vertex in enumerate(face.verts):
--             uv = face.loops[i][self.uv_layer].uv
--             # rotate vertex into new texture coords (in meters)
--             v = np.array(vertex.co)

--             v = v - trans
--             geom.rotate(v,rot)
--             x = v[dims[1]] - xrange[0]
--             y = v[dims[2]] - yrange[0]
--             # x,y => uv.x,uv.y
--             uv.x = x / widthm
--             uv.y = 1 - (y / heightm)

-- Algo:

-- 1) get closest poses: (see func. get_closest_poses())

-- 2) find matrix to rot


-- 3) loop through the 2D texture.
--  a) inverse from texture coords to global
--  b) loop through closest poses
--      i) get uv of global coordinate in the pose
--     ii) check obvious out of bounds (including a buffer at top and
--         bottom 0px for matterport textures)

--             if (px < 0) or (px >= timg.shape[1]) \
--                or (py < self.vertbuffer)    \
--                or (py >= (timg.shape[0] - self.vertbuffer)):

--    iii) check occlusion (look up in ray traced pose mask)
--     iv) compute alpha (mixing) for this pose. (see. func. compute_alpha())
--  c) blend colors from different poses using alpha (use max per pixel as we go for now)

-- 4) save out new texture file

-- 5) Remap UVs: rotate each vertex into the coordinates of the new texture
--    (See. func remap_uvs)
