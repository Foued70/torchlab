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

-- more args FIXME become arguments
ppm        = 100 -- pixels per meter
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


--   -- idea: projecting whole wireframe onto pose would help with
--      alignment of all faces at once (Reduce human alignment
--      tweaks).
--
-- thought to help debug the image stitching: overlay the wireframe of
-- the regeom on the texture.
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
            -- printf("step: %f,%f,%f",step[1],step[2],step[3])
            for s = 0,len,mpp do 
               -- draw verts first
               local u,v,x,y = util.pose.globalxyz2uv(p,i,pvert)
               -- printf("u: %f v: %f x: %f y %f", u, v, x, y)
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

function save_all_wireframes()
   for pi = 1,poses.nposes do 
      local wimage = draw_wireframe(poses,pi,target)
      image.display(wimage)
      -- save
      local wimagename = poses[pi]:gsub(".jpg","_wireframe.png")
      printf("Saving: %s", wimagename)
      image.save(wimagename,wimage)
   end
end

-- TODO 
-- 1) redo retexture (no optimisations just straight up) and write obj file.
--   a) debug: find UV in pose for a texture (test pose globalxyz2uv function)
--   b) draw wireframe on pose. [DONE]


-- Main Functions:

-- + ----------------
-- get_closest_poses() 
-- for a given face select the best poses we will use to color it.
-- input: poses <p> , face ids <fid>, object <obj>
function get_closest_poses(p,fid,obj,debug)

   --  a) find poses which are on the correct side of the face normal

   local normal = obj.normals[fid]
   local d      = obj.d[fid]

   local dist_to_plane = torch.mv(p.xyz,normal) + d

   local wrong_side = torch.lt(dist_to_plane,0):double()
   local invalid    = torch.sum(wrong_side)
   if debug then printf("[%d] wrong side: %d", fid,invalid) end
   wrong_side:mul(1e6):add(1) 
   
   -- b) who are closest (to the face center)
   local center = obj.centers[fid]
   local dirs   = p.xyz:clone()

   for pi = 1,p.nposes do 
      dirs[pi]:add(-1,center)
   end
    
   local len = dirs:norm(2,2):squeeze():cmul(wrong_side)
   local slen,sidx = torch.sort(len)
   if debug then 
      printf("[%d] top5 : %d: %2.2f %d: %2.2f %d: %2.2f %d: %2.2f %d: %2.2f",
             fid, 
             sidx[1],slen[1],
             sidx[2],slen[2],
             sidx[3],slen[3],
             sidx[4],slen[4],
             sidx[5],slen[5])
   end
   --  Reconsider: decisions c and d happen when coloring
   --  c) reject poses where the camera is too close.  Can reject just on
   --     the face center as we prefer whole face to be colored by a
   --     single pose.
   
   --  d) make sure that at least one vertex of face falls within the
   --     poses view. (not sure we need this)

      return sidx:narrow(1,1,p.nposes-invalid)
end

function test_get_closest_poses() 
   for fid = 1,target.face_verts:size(1) do 
      get_closest_poses(poses,fid,target,true)
   end
end

-- + ----------------
-- face_to_texture_transform_and_dimensions() 

-- find rotatation and translation for a virutal camera centered on
-- the face and the dimensions of a texture map
function face_to_texture_transform_and_dimension(fid,obj,debug)
   local face_verts = obj.face_verts[fid]
   local normal     = obj.normals[fid]

   --  a) find translation (point closest to origin of longest edge)
   local eid = 1
   local maxedge = torch.norm(face_verts[1] - face_verts[-1])
   for vi = 2,face_verts:size(1) do 
      local elen = torch.norm(face_verts[vi] - face_verts[vi-1])
      if maxedge < elen then 
         maxedge = elen
         eid = vi
      end
   end
   local pid = eid - 1
   if pid == 0 then pid = face_verts:size(1) end
   local trans = face_verts[pid]:clone()
   local longedge = face_verts[eid] - trans
   longedge = geom.normalize(longedge)

   -- align largest dimension of the plane normal
   local nrot,zdim = geom.largest_rotation(normal)

   -- align longest edge which is in a plane orthogonal to the new
   --zaxis, and needs to be rotated to the xaxis around the zaxis
   local rnlongedge = geom.normalize(geom.rotate_by_quat(longedge,nrot))
   local erot,xdim  = geom.largest_rotation(rnlongedge)
   if debug then
      printf(" -- nrot: %2.4f %2.4f %2.4f %2.4f",nrot[1],nrot[2],nrot[3],nrot[4])
      printf(" -- erot: %2.4f %2.4f %2.4f %2.4f",erot[1],erot[2],erot[3],erot[4])
   end
   local rot = geom.quat_product(erot,nrot)

   local ydim = 6 - (xdim+zdim)
   local dims = torch.Tensor({zdim,xdim,ydim})
   --  b) find dimensions of the texture to be created
   local v = geom.rotate_by_quat(face_verts[1] - trans,rot)
   -- range is min,max,range
   local xrange = torch.Tensor(3):fill(v[xdim])
   local yrange = torch.Tensor(3):fill(v[ydim])
   for vi = 2,face_verts:size(1) do 
      v = geom.rotate_by_quat(face_verts[vi] - trans,rot)
      if (yrange[1] > v[ydim]) then yrange[1] = v[ydim] end
      if (yrange[2] < v[ydim]) then yrange[2] = v[ydim] end
      if (xrange[1] > v[xdim]) then xrange[1] = v[xdim] end
      if (xrange[2] < v[xdim]) then xrange[2] = v[xdim] end
   end
   xrange[3] = xrange[2] - xrange[1] 
   yrange[3] = yrange[2] - yrange[1] 

   return rot,trans,dims,xrange,yrange
end

function test_face_to_texture()
   for fid = 1,target.face_verts:size(1) do
      printf("[%03d]",fid)
      n = target.normals[fid]
      printf(" -- nrm: %2.4f %2.4f %2.4f",n[1],n[2],n[3])
      r,t,d,x,y = face_to_texture_transform_and_dimension(fid,target)
      printf(" -- rot: %2.4f %2.4f %2.4f %2.4f",r[1],r[2],r[3],r[4])
      print(geom.rotation_matrix(r))
      printf(" -- trs: %2.4f %2.4f %2.4f",t[1],t[2],t[3])
      printf(" -- dim: %d %d %d",d[1],d[2],d[3])
      printf(" -- xrg: %2.4f - %2.4f",x[1],x[2])
      printf(" -- yrg: %2.4f - %2.4f",y[1],y[2])
      
   end
end

-- + ----------------
-- compute_alpha(dir,d,norm)        
--  prefer smaller angle w/ respect to norm
-- (normalize with pi/2 as that is the biggest angle)
function compute_alpha(dir,d,norm,debug)
   if debug then printf(" - d: %f", d) end
   if (d < mindist) or (d > maxdist) then return 0 end
   local a = 1 - torch.acos(torch.dot(-dir,norm))/pi2
   if debug then printf(" - a: %f",a) end
   -- reject obvious wrong
   if (torch.abs(a) > pi2) then return 0 end
   -- do ramp to ideal dist
   if (d < idealdist) then
      return a*(d*fin_a + fin_b)
   else 
      return a*(d*fout_a + fout_b)
   end
end

-- + ----------------
-- create_uvs()
function create_uvs (fid,obj,rot,trans,dims,xrange,yrange)
   local face_verts = obj.face_verts
   local uv = torch.Tensor(face_verts:size(1),2)

   for vi = 1,face_verts:size(1) do 
      local vtrans = face_verts[vi] - trans
      vtrans = geom.rotate_by_quat(vtrans,rot)
      uv[vi][1] =      (vtrans[dims[2]] - xrange[1])/xrange[3]
      uv[vi][2] = 1 - ((vtrans[dims[3]] - yrange[1])/yrange[3])
   end
   return uv
end


function retexture (fid,obj)
   
   -- Retexture Algo:
   
   -- 1) get closest poses: 
   local pose_idx = get_closest_poses(poses,fid,obj)
   pose_idx = pose_idx:narrow(1,1,nposes)

   -- 2) find rotation and translation to texture coords and dimensions of texture
   local rot,trans,dims,xrange,yrange = face_to_texture_transform_and_dimension(fid,obj)
   --  a) need rotation from texture to global
   local rotT = geom.quat_conjugate(rot)

   -- 3) create the texture image which we will color, and temp variables
   local widthpx  = math.floor(xrange[3] * ppm + 1)
   local heightpx = math.floor(yrange[3] * ppm + 1)
   local dx = xrange[3]/widthpx
   local dy = yrange[3]/heightpx

   printf("w: %f h: %f dx: %f dy: %f",widthpx,heightpx,dx,dy)

   -- make the temporary per pose mixing alpha and color channels
   local normal = obj.normals[fid] 
   local alpha  = torch.zeros(nposes)
   local color  = torch.zeros(nposes,3)
   local v      = torch.zeros(3)

   fimg = torch.zeros(3,heightpx,widthpx)

   -- 4) loop through the 2D texture. FIXME optimize using ray packets.
   local y = yrange[1]
   for h = 1,heightpx do 
      local x = xrange[1]
      for w = 1,widthpx do
         -- place point in texture coordinate
         v[dims[2]] = x
         v[dims[3]] = y
         v[dims[1]] = 0

         --  inverse from texture coords to global
         v = geom.rotate_by_quat(v,rotT) + trans
         local found = 0
         -- loop through closest poses
         for pi = 1,pose_idx:size(1) do 
            local pid  = pose_idx[pi] 
            local timg = poses.images[pid] 
            local pt   = poses.xyz[pid]

            --  get uv of global coordinate in the pose
            local pu,pv,px,py = util.pose.globalxyz2uv(poses,pid,v)

            -- check obvious out of bounds (including a buffer at top
            -- and bottom 0px for matterport textures)
            if (((px < 0) or (px >= timg:size(3))) or 
             (py < vertbuffer) or (py >= (timg:size(2) - vertbuffer))) then
               printf("[%d] out of range skipping",pid)
            else
               -- check occlusion (look up in ray traced pose mask)
               -- FIXME 
               --  compute alpha (mixing) for this pose. (see. func. compute_alpha())
               local dir  = v - pt -- from pose to surface
               local dist = dir:norm()
               dir = dir * (1/dist)

               found = found + 1
               color[found] = timg[{{},py,px}]
               alpha[found] = compute_alpha(dir,dist,normal)
            end
            if found >= nposes then break end
         end

         -- 5) blend colors from different poses using alpha (use max per
         --    pixel as we go for now)
         local s = alpha:sum()
         if (s > 0) then 
            local pmax,pid = alpha:max(1)
            pid = pid[1]
            fimg[{{},h,w}] = color[pid]
         end
         -- done with this pixel
         color:fill(0)
         alpha:fill(0)
         x = x + dx
      end -- end w
      y = y + dy
   end -- end h

   -- 6) save out new texture file
   image.display{image=fimg,min=0,max=1}
   local fname = string.format("out-%05d.png",fid)
   printf("Saving: %s", fname)
   image.save(fname,fimg)

   -- 7) Remap UVs: rotate each vertex into the coordinates of the new texture
   --    (See. func remap_uvs)
end
