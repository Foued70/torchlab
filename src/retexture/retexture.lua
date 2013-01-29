require 'torch'
require 'sys'
require 'paths'
require 'math'
local util = require 'util'


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
posedir     = paths.dirname(posefile)

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
   target = loadcache(targetfile,targetcache,util.obj.load,10)
end

-- more args FIXME become arguments and make local
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
   
   if (p.nposes == invalid) then
      return nil
   end
   
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
   for fid = 1,target.nfaces do
      get_closest_poses(poses,fid,target,true)
   end
end

-- + ----------------
-- face_to_texture_transform_and_dimensions()

-- find rotatation and translation for a virutal camera centered on
-- the face and the dimensions of a texture map
function face_to_texture_transform_and_dimension(fid,obj,debug)
   local face_verts = obj.face_verts[fid]
   local nverts     = obj.nverts_per_face[fid]
   local normal     = obj.normals[fid]

   --  a) find translation (point closest to origin of longest edge)
   local eid = 1
   local maxedge = torch.norm(face_verts[1] - face_verts[nverts])
   local e2 = nil
   for vi = 2,nverts do
      local v1 = face_verts[vi]
      local v2 = face_verts[vi-1]
      local e1 = v1 - v2
      local elen = torch.norm(e1)
      if debug and e2 then 
         local en = torch.cross(v1,v2)
         printf("vertex normal: %f %f %f",en[1],en[2],en[3])
      end
      if maxedge < elen then
         maxedge = elen
         eid = vi
      end
      e2 = e1
   end
   local pid = eid - 1
   if pid == 0 then pid = nverts end
   local trans = face_verts[pid]:clone()
   local longedge = face_verts[eid] - trans

   -- make sure we translate by the edge closest to origin
   if (trans:norm() > face_verts[eid]:norm()) then
      if debug then print("Swapping translation") end
      longedge = trans - face_verts[eid] 
      trans:copy(face_verts[eid])
   end
   
   if debug then 
      printf("longedge: %d -> %d", pid, eid)
   end

   -- align largest dimension of the plane normal
   local nrot,ndim = geom.largest_rotation(normal)
   
   -- align longest edge which is in a plane orthogonal to the new
   -- zaxis, and needs to be rotated to the xaxis around the zaxis
   local nrot_longedge = geom.rotate_by_quat(longedge,nrot)

   local erot,edim  = geom.largest_rotation(nrot_longedge)

   -- combine the rotations into a single rotation
   local rot = geom.quat_product(erot,nrot)
   
   local ydim = 6 - (edim+ndim)
   local dims = torch.Tensor({ndim,edim,ydim})

   if debug then
      printf("   Orig normal: %f %f %f",normal[1],normal[2],normal[3])
      local n = geom.rotate_by_quat(normal,nrot)
      printf("Rotated normal: %f %f %f",n[1],n[2],n[3])
      printf(" --  long edge: %f %f %f",longedge[1],longedge[2],longedge[3])
      printf(" --  nrot edge: %f %f %f",nrot_longedge[1],nrot_longedge[2],nrot_longedge[3])
      local enrot_longedge  = geom.rotate_by_quat(nrot_longedge,erot)
      printf(" -- enrot edge: %f %f %f",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])
      enrot_longedge  = geom.rotate_by_quat(longedge,rot)
      printf(" -- enrot edge: %f %f %f",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])
      
      printf(" -- [%d]  nrot: %f %f %f %f",ndim,nrot[1],nrot[2],nrot[3],nrot[4])
      printf(" -- [%d]  erot: %f %f %f %f",edim,erot[1],erot[2],erot[3],erot[4])
      printf(" -- [%d]   rot: %f %f %f %f",ydim,rot[1],rot[2],rot[3],rot[4])
   end

   


   --  b) find dimensions of the texture to be created
   local v = geom.rotate_by_quat(face_verts[1] - trans,rot)
   -- range is min,max,range
   local xrange = torch.Tensor(3):fill(v[edim])
   local yrange = torch.Tensor(3):fill(v[ydim])
   for vi = 2,nverts do
      v = geom.rotate_by_quat(face_verts[vi] - trans,rot)
      if (yrange[1] > v[ydim]) then yrange[1] = v[ydim] end
      if (yrange[2] < v[ydim]) then yrange[2] = v[ydim] end
      if (xrange[1] > v[edim]) then xrange[1] = v[edim] end
      if (xrange[2] < v[edim]) then xrange[2] = v[edim] end
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
   local face_verts = obj.face_verts[fid]
   local nverts     = obj.nverts_per_face[fid]
   local uv         = obj.uv[fid]

   for vi = 1,nverts do
      local vtrans = face_verts[vi] - trans
      vtrans = geom.rotate_by_quat(vtrans,rot)
      uv[vi][1] =      (vtrans[dims[2]] - xrange[1])/xrange[3]
      uv[vi][2] = 1 - ((vtrans[dims[3]] - yrange[1])/yrange[3])
   end
   return uv
end


function retexture (fid,obj,debug)

   -- Retexture Algo:
   -- 1) get closest poses:
   local pose_idx = get_closest_poses(poses,fid,obj)
   if not pose_idx then
      printf("face: %d -- No valid poses found",fid)
      return 
   end
   -- just look at nposes (5) poses per face
   if (pose_idx:size(1) > nposes) then
      pose_idx = pose_idx:narrow(1,1,nposes)
   end
   -- 2) find rotation and translation to texture coords and dimensions of texture
   local rot,trans,dims,xrange,yrange = 
      face_to_texture_transform_and_dimension(fid,obj)
   --  a) need rotation from texture to global
   local rotT = geom.quat_conjugate(rot)

   -- 3) create the texture image which we will color, and temp variables
   local widthpx  = math.floor(xrange[3] * ppm + 1)
   local heightpx = math.floor(yrange[3] * ppm + 1)
   local dx = xrange[3]/widthpx
   local dy = yrange[3]/heightpx

   printf("face: %d texture w: %d h: %d dx: %f dy: %f",fid,widthpx,heightpx,dx,dy)

   -- make the temporary per pose mixing alpha and color channels
   local normal = obj.normals[fid]
   local alpha  = torch.zeros(nposes)
   local color  = torch.zeros(nposes,3)
   local v      = torch.zeros(3)
   if not obj.uv then
      obj.uv = torch.Tensor(obj.face_verts:size(1),obj.face_verts:size(2),2)
   end
   local uv = obj.uv
   if not obj.textures then
      obj.textures = {}
   end
   -- texture image
   local fimg = torch.zeros(3,heightpx,widthpx)

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
               if debug then printf("[%d] out of range skipping",pid) end
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

   -- 6) store new texture file
   obj.textures[fid] = fimg
   image.display{image=fimg,min=0,max=1}

   local fname  = paths.basename(targetfile):gsub(".obj",string.format("_face%05d.png",fid))
   printf(" - Saving: texture %s", fname)
   image.save(fname,fimg)

   -- 7) Remap UVs: rotate each vertex into the coordinates of the new texture
   create_uvs(fid,obj,rot,trans,dims,xrange,yrange)
end

function retexture_all()
   for fid = target.nfaces-1,target.nfaces do
      sys.tic()
      retexture(fid,target)
      printf(" - textured in %2.2fs",sys.toc())
   end
   local objfile  = paths.basename(targetfile)
   local mtlfile  = objfile:gsub(".obj",".mtl")
   local textfile = objfile:gsub(".obj","")
   printf("Writing: %s", objfile)
   printf("Writing: %s", mtlfile)
   util.obj.save(target,objfile,mtlfile,textfile)
end

retexture_all()