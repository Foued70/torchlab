-- TODO: instead of posefile and targetfile, pass scan.lua or folder with scan.lua. 

require 'torch'
require 'sys'
require 'paths'
require 'math'

local Poses = require 'Poses'
local loader = require 'util.loader'
local geom = require 'util.geom'

-- TODO: deal with output dir -- arc?
local output_dir = paths.concat(paths.dirname(paths.thisfile()), 'output')
sys.execute("mkdir -p " .. output_dir)

local Textures = Class()

local pi = math.pi
local pi2 = pi/2

function Textures:__init(posefile, targetfile, opts)
  if not posefile or not targetfile then error('arguments invalid') end
  if not paths.filep(posefile) or not paths.filep(targetfile) then 
    error('pose file or target file does not exist') 
  end  
  
  self.targetfile = targetfile
  self:load_target() 
  
  self.posefile = posefile
  self.poses = loader(posefile, Poses.new)
  
  self.textures = {}
  self.faces = {} -- store some info about faces
  
  local defaults = {
    scale = 4, -- scale at which to process 4 = 1/4 resolution
    ppm = 150,  -- pixels per meter    
    nposes = 8, -- max number of poses to consider per texture
    mindist = 0.7, -- min distance to scanner
    ideal = 1.5, -- meters for fade
    maxdist = 50,
    vertbuffer = 0 -- how close in pixels to edge of texture do we accept
  }
  
  opts = opts or {}
  for k, v in pairs(defaults) do
    self[k] = opts[k] or v
  end
  
  self.scale_inv = 1/self.scale

  -- simple linear fadein/out eqn.
  self.idealdist   =  self.ideal + self.mindist

  -- line 1 from (mindist,0) -> (mindist+ideal,1)
  self.fin_a   =  1/self.ideal
  self.fin_b   =  1 - self.fin_a * self.idealdist

  -- line 2 from (mindist+ideal,1) -> (maxdist,0)
  self.fout_a  = -1/(self.maxdist - self.idealdist)
  self.fout_b  =  1 - self.fout_a * self.idealdist
  
  self:load_occlusions()
  -- self:load_masks()  TODO: masks ?  
end

function Textures:load_target()
  local obj = loader(self.targetfile, require('util.Obj').new)
  -- delete some properties, we're going to remake these    
  obj.unified_verts = torch.Tensor(obj.n_faces*obj.n_gon, 9):fill(1)
  obj.uvs = torch.Tensor(obj.n_faces*obj.n_gon,2):fill(1)
  obj.n_uvs = 0
  obj.submeshes = {}
  obj.materials = {}
  self.target = obj
end

function Textures:load_occlusions()
  -- assumes occlusions have been calculated already
  local occlusions = retex.Occlusions.new(self.posefile, self.targetfile, self.scale)
  self.occlusions = occlusions:get()
end

-- + ----------------
-- get_closest_poses()
-- for a given face select the best poses we will use to color it.
-- input: face ids <fid>
-- FIXME redo this function.  Trace ray to each pose.
function Textures:get_closest_poses(fid,debug)  
  local obj = self.target
  local poses = self.poses
  
  --  a) find poses which are on the correct side of the face normal
  local normal = obj.face_normals[fid]
  local d = obj.face_center_dists[fid]
  
  local dist_to_plane = torch.mv(poses.xyz,normal) + d

  local wrong_side = torch.lt(dist_to_plane,0):double()
  local invalid    = torch.sum(wrong_side)
  
  log.trace(invalid, 'invalid poses for face', fid)
  
  if (poses.nposes == invalid) then 
    return nil  -- all the poses are on the wrong side
  end
   
  wrong_side:mul(1e6):add(1)

  -- b) who are closest (to the face center)
  local center = obj.face_centers[fid]
  local dirs   = poses.xyz:clone()

  for pi = 1,poses.nposes do
    dirs[pi]:add(-1,center)
  end

  local len = dirs:norm(2,2):squeeze():cmul(wrong_side)
  local slen,sidx = torch.sort(len)
  if debug then
    log.trace('face', fid, ' - sidx/slen - ', 
      sidx[1],slen[1], 
      sidx[2],slen[2],
      sidx[3],slen[3],
      sidx[4],slen[4],
      sidx[5],slen[5])
   end
   
   --  Reconsider: decisions c and d happen when coloring.
   --  c) reject poses where the camera is too close.  Can reject just on
   --     the face center as we prefer whole face to be colored by a
   --     single pose.

   --  d) make sure that at least one vertex of face falls within the
   --     poses view. (not sure we need this) reject some fully occluded poses.
   
   -- just look at nposes per face
   local limit = math.min(self.nposes, poses.nposes-invalid)
   return sidx:narrow(1,1,limit)
end

function Textures:test_get_closest_poses()
  for fid = 1,self.target.n_faces do
    get_closest_poses(fid,true)
  end
end

-- + ----------------
-- face_to_texture_transform_and_dimensions()
-- find rotatation and translation for a virutal camera centered on
-- the face and the dimensions of a texture map
function Textures:face_to_texture_transform_and_dimension(fid,debug)
  if not self.faces[fid] then
    log.trace('calc texture vals for face', fid)
    local obj = self.target
  
    local face_verts = obj.face_verts[fid]
    local nverts     = obj.n_verts_per_face[fid]
    local normal     = obj.face_normals[fid]

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
        log.trace("vertex normal:", en[1],en[2],en[3])
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
      if debug then log.trace("Swapping translation") end
      longedge = trans - face_verts[eid] 
      trans:copy(face_verts[eid])
    end
   
    if debug then log.trace("longedge:", pid, eid) end

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
      log.trace("   Orig normal:", normal[1],normal[2],normal[3])
      local n = geom.rotate_by_quat(normal,nrot)
      log.trace("Rotated normal:",n[1],n[2],n[3])

      log.trace(" --  long edge:",longedge[1],longedge[2],longedge[3])
      log.trace(" --  nrot edge:",nrot_longedge[1],nrot_longedge[2],nrot_longedge[3])
      local enrot_longedge  = geom.rotate_by_quat(nrot_longedge,erot)    
      log.trace(" -- enrot edge:",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])
      enrot_longedge  = geom.rotate_by_quat(longedge,rot)
      log.trace(" -- enrot edge:",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])

      log.trace("-- dim: ", ndim, "nrot:", nrot[1],nrot[2],nrot[3],nrot[4])
      log.trace("-- dim:", edim, "erot:",edim,erot[1],erot[2],erot[3],erot[4])
      log.trace("-- dim:", ydim, "rot:",ydim,rot[1],rot[2],rot[3],rot[4])
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

    self.faces = {}    
    self.faces[fid] = {rot, trans, dims, xrange, yrange}
  end
  
  return unpack(self.faces[fid])
end

function Textures:test_face_to_texture()
   for fid = 1,self.target.face_verts:size(1) do
      printf("[%03d]",fid)
      n = self.target.face_normals[fid]
      printf(" -- nrm: %2.4f %2.4f %2.4f",n[1],n[2],n[3])
      r,t,d,x,y = face_to_texture_transform_and_dimension(fid, true)
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
function Textures:compute_alpha(dir,d,norm,debug)  
  if debug then log.trace('computing alpha - dir', dir, 'd', d, 'norm', norm) end
  if (d < self.mindist) or (d > self.maxdist) then return 0 end   

  -- angle 1 == perpendicular.
  local a = 1 - torch.acos(torch.dot(-dir,norm))/pi2
  if debug then log.trace("a:",a) end
  -- reject obvious wrong
  if (torch.abs(a) > pi2) then return 0 end
  -- do ramp to ideal dist
  if (d < self.idealdist) then
    return a*(d*self.fin_a + self.fin_b)
  else
    return a*(d*self.fout_a + self.fout_b)
  end
end

-- Remap UVs: rotate each vertex into the coordinates of the new texture
function Textures:create_uvs(fid, debug)  
  log.trace(obj.n_verts_per_face[fid], 'uvs for face', fid)  
  local rot,trans,dims,xrange,yrange = self:face_to_texture_transform_and_dimension(fid, debug)  
  local obj = self.target  
  local face_verts = obj.face_verts[fid]
  
  for vi = 1, obj.n_verts_per_face[fid] do
    local vtrans = face_verts[vi] - trans
    vtrans = geom.rotate_by_quat(vtrans,rot)
    obj.n_uvs = obj.n_uvs + 1
    obj.uvs[obj.n_uvs][1] = (vtrans[dims[2]] - xrange[1])/xrange[3]
    obj.uvs[obj.n_uvs][2] = 1 - ((vtrans[dims[3]] - yrange[1])/yrange[3])
    obj.faces[fid][vi][3] = obj.n_uvs
  end  
end

function Textures:range_to_texture_dimensions(xrange, yrange, debug)  
  local widthpx  = math.floor(xrange * self.ppm + 1)
  local heightpx = math.floor(yrange * self.ppm + 1)
  local dx = xrange/widthpx
  local dy = yrange/heightpx
  if debug then log.trace("texture dimensions - w:", widthpx, "h:", heightpx, "dx:", dx, "dy:", dy) end
  return widthpx, heightpx, dx, dy
end

function Textures:make_img(fid, debug)
  local obj         = self.target
  local nposes      = self.nposes
  local poses       = self.poses
  local occlusions  = self.occlusions
  local scale_inv   = self.scale_inv
  
  -- Retexture Algo:
  -- 1) get closest poses:
  local closest_poses = self:get_closest_poses(fid, debug)
  if not closest_poses then
    log.trace('No valid poses found for face', fid)
    return 
  end
  
  -- 2) find rotation and translation to texture coords and dimensions of texture
  local rot,trans,dims,xrange,yrange = self:face_to_texture_transform_and_dimension(fid, debug)
    
  --  a) need rotation from texture to global
  local rotT = geom.quat_conjugate(rot)

  -- 3) create the texture image which we will color, and temp variables
  local widthpx, heightpx, dx, dy = self:range_to_texture_dimensions(xrange[3], yrange[3], debug)
  
  log.trace(widthpx, 'x', heightpx, 'image for face', fid)
  -- make the temporary per pose mixing alpha and color channels
  local normal = obj.face_normals[fid]
  local alpha  = torch.zeros(nposes)
  local color  = torch.zeros(nposes,3)
  local v      = torch.zeros(3)

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
      for pi = 1,closest_poses:size(1) do
        local pid  = closest_poses[pi]
        local pose = poses[pid]
        local timg = pose.image
        local pt   = pose.xyz
        local p_occ = nil
        
        if occlusions then
          p_occ = occlusions[pid]
        end
        
        --  get uv of global coordinate in the pose
        local pu,pv,px,py = pose:globalxyz2uv(v)
        if debug then log.trace("pid:", pid, 'py', py, 'px', px) end
        
        -- check obvious out of bounds (including a buffer at top
        -- and bottom 0px for matterport textures)
        if (px < 1) or (px >= timg:size(3)) then
          log.trace(px, "px out of range for pose", pid, "(should not happen)")
        elseif (py < 1) or (py >= timg:size(2)) then
          if debug then log.trace(py, "py out of range for pose", pid) end
        -- elseif (use_masks and (pose.mask[py][px] < 1)) then
        --   printf("pose[%d] masked at %f, %f", pid, py, px)
        else  
          -- compute alpha (mixing) for this pose. (see. func. compute_alpha())
          local dir  = v - pt -- from pose to surface
          local dist = dir:norm()
          local not_occluded = true
          -- check occlusion (look up in ray traced pose mask)
          if p_occ then
            local opx = math.max(1,math.floor(px*scale_inv + 0.5))
            local opy = math.max(1,math.floor(py*scale_inv + 0.5))
            local od  = p_occ[opy][opx]
            -- only is falsified 
            not_occluded = torch.abs(dist - od) < 0.2
            -- printf("occluded: %s dist: %f od: %f",not not_occluded,dist,od)
          end
          if not_occluded then
            dir = dir * (1/dist) 
            found = found + 1
            color[found] = timg[{{},py,px}]
            alpha[found] = self:compute_alpha(dir,dist,normal,debug)
          end
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
  -- not sure about saving to disk, or saving in object.
  self.textures[fid] = fimg
end

-- get file path for a face's texture
function Textures:file(fid)
  local name = paths.basename(self.targetfile):gsub(".obj$", "_face-"..fid..".png")
  return paths.concat(output_dir, name)
end

-- save the texture for a face and update the materials and submeshes
function Textures:save_img(fid)    
  local texture_img = self.textures[fid]  
  local material = {
    name = "face"..fid,
    diffuse = {0.5, 0.5, 0.5, 1}    
  }
  if texture_img then
    local img_file = self:file(fid)
    log.trace('Saving texture for face', fid, img_file)
    win = image.display{image=texture_img,min=0,max=1,win=win}    
    image.save(img_file, texture_img)
    material.diffuse_tex_path = paths.basename(img_file)
  else
    log.trace('No texture for face', fid)
  end
  table.insert(self.target.materials, material)
  table.insert(self.target.submeshes, {fid, fid, #self.target.materials})
end

-- update obj properties and save it
function Textures:save_obj()
  log.trace('saving obj')
  local obj = self.target
  local faces = obj.faces  
  local verts = obj.verts
  local uvs = obj.uvs
  local n_verts_per_face = obj.n_verts_per_face
  
  local vert_cache = {}
  local unified_verts_idx = 0
  
  for fid=1, obj.n_faces do  
    for vert_i=1, n_verts_per_face[fid] do
      local vert_idx = faces[fid][2]
      local uv_idx = faces[fid][3]
      
      local idx = vert_cache[{vert_idx, uv_idx}]
      if not idx then
        unified_verts_idx = unified_verts_idx + 1
        idx = unified_verts_idx
        vert_cache[{vert_idx, uv_idx}] = idx
        
        unified_verts[{idx, {1, 3}}] = verts[vert_idx]:narrow(1, 1, 3)
        unified_verts[{idx, {5, 6}}] = uvs[uv_idx]
      end
    end
  end
  
  local trimmed_uvs = torch.Tensor(obj.n_uvs, 2)
  trimmed_uvs[{{1, obj.n_uvs}}] = uvs[{{1, obj.n_uvs}}]
  
  local trimmed_unified_verts = torch.Tensor(unified_verts_idx, 9)
  trimmed_unified_verts[{{1, unified_verts_idx}}] = unified_verts[{{1, unified_verts_idx}}]
  
  obj.unified_verts = trimmed_unified_verts
  obj.uvs = trimmed_uvs
  obj.submeshes = torch.IntTensor(obj.submeshes)
  
  local objfile  = paths.concat(output_dir, paths.basename(self.targetfile))
  local mtlfile  = objfile:sub(".obj$",".mtl")
  log.trace('Saving obj and mtl', objfile, mtlfile)
  obj.save(objfile, mtlfile)
end

function Textures:make()     
  for fid = 1, self.target.n_faces do
    self:make_img(fid)
    self:create_uvs(fid)
    self:save_img(fid) 
  end
  self:save_obj()
end