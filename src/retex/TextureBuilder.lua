-- takes a scan and writes .obj, .mtl, and various .pngs to file sys
-- also updates the obj instance for direct viewer display
-- example usage:
-- tex = retex.Textures.new(scan, {ppm = 300})
-- tex:make()

local loader = require '../data/loader'

local TextureBuilder = Class()

local pi  = math.pi
local pi2 = pi/2

function TextureBuilder:__init(scan, opts)
   if not scan then error('arguments invalid') end

   self.model = scan

   self.textures = {}
   self.faces    = {} -- store some info about faces

   local defaults = {
      scale      = 4,   -- scale at which to process 4 = 1/4 resolution
      ppm        = 100, -- pixels per meter
      mindist    = 0.7, -- min distance to scanner
      ideal      = 1.5, -- meters for fade
      maxdist    = 50,
      vertbuffer = 0    -- how close in pixels to edge of texture do we accept
  }

  opts = opts or {}
  for k, v in pairs(defaults) do
    self[k] = opts[k] or v
  end

  -- TODO move this to a geometry blend function
  self.scale_inv = 1/self.scale

  -- simple linear fadein/out eqn.

  -- line 1 from (mindist,0) -> (ideal,1)
  self.fin_a   =  1/ (self.ideal - self.mindist)
  self.fin_b   =  -1 * (self.fin_a * self.mindist)

  -- line 2 from (ideal,1) -> (maxdist,0)
  self.fout_a  = -1/(self.maxdist - self.ideal)
  self.fout_b  =  self.fout_a * self.maxdist

  -- self:load_occlusions()
  -- self:load_masks()  TODO: masks ?
end

-- + ----------------
-- face_to_texture_transform_and_dimensions()
-- find rotatation and translation for a virutal camera centered on
-- the face and the dimensions of a texture map
function TextureBuilder:face_to_texture_transform_and_dimension(fid,debug)
  if not self.faces[fid] then
    local model = self.model

    local face_verts = model.face_verts[fid]:narrow(2,1,3)
    local nverts     = model.n_verts_per_face[fid]
    local normal     = model.face_normals[fid]
    --  a) find translation (point closest to origin of longest edge)
    local eid = 1
    local maxedge = torch.norm(face_verts[1] - face_verts[nverts])
    local e2 = nil
    for vi = 2,nverts do
      local v1 = face_verts[vi]
      local v2 = face_verts[vi-1]
      local e1 = v1 - v2
      local elen = torch.norm(e1)
      if maxedge < elen then
        maxedge = elen
        eid = vi
      end
    end

    local pid = eid - 1
    if pid == 0 then pid = nverts end
    local trans    = face_verts[pid]:clone()
    local longedge = face_verts[eid] - trans

    -- make sure we translate by the edge closest to origin
    if (trans:norm() > face_verts[eid]:norm()) then
      if debug then log.trace("Swapping translation") end
      longedge = trans - face_verts[eid]
      trans:copy(face_verts[eid])
    end

    if debug then 
       log.trace("longedge:", pid, eid) 
       log.trace("    from:", face_verts[pid][1], face_verts[pid][2], face_verts[pid][3])
       log.trace("      to:", face_verts[eid][1], face_verts[eid][2], face_verts[eid][3])
    end

    -- align largest dimension of the plane normal
    local nrot,ndim = geom.rotation.largest(normal)

    -- align longest edge which is in a plane orthogonal to the new
    -- zaxis, and needs to be rotated to the xaxis around the zaxis
    local nrot_longedge = geom.quaternion.rotate(nrot,longedge)

    local erot,edim  = geom.rotation.largest(nrot_longedge)

    -- combine the rotations into a single rotation
    local rot = geom.quaternion.product(erot,nrot)

    local ydim = 6 - (edim+ndim)
    local dims = torch.Tensor({ndim,edim,ydim})

    if debug then
      log.trace("   Orig normal:", normal[1],normal[2],normal[3])
      local n = geom.quaternion.rotate(nrot,normal)
      log.trace("Rotated normal:",n[1],n[2],n[3])

      log.trace(" --  long edge:",longedge[1],longedge[2],longedge[3])
      log.trace(" --  nrot edge:",nrot_longedge[1],nrot_longedge[2],nrot_longedge[3])
      local enrot_longedge  = geom.quaternion.rotate(erot,nrot_longedge)
      log.trace(" -- enrot edge:",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])
      enrot_longedge  = geom.quaternion.rotate(rot,longedge)
      log.trace(" -- enrot edge:",enrot_longedge[1],enrot_longedge[2],enrot_longedge[3])

      log.trace("-- dim: ", ndim, "nrot:",ndim,nrot[1],nrot[2],nrot[3],nrot[4])
      log.trace("-- dim:", edim, "erot:",edim,erot[1],erot[2],erot[3],erot[4])
      log.trace("-- dim:", ydim, "rot:",ydim,rot[1],rot[2],rot[3],rot[4])
    end

    --  b) find dimensions of the texture to be created
    local v = geom.quaternion.rotate(rot,face_verts[1] - trans)
    -- range is min,max,range
    local xrange = torch.Tensor(3):fill(v[edim])
    local yrange = torch.Tensor(3):fill(v[ydim])
    for vi = 2,nverts do
      v = geom.quaternion.rotate(rot,face_verts[vi] - trans)
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

function TextureBuilder:test_face_to_texture(fid)
   printf("[%03d]",fid)
   n = self.model.face_normals[fid]
   printf(" -- nrm: %2.4f %2.4f %2.4f",n[1],n[2],n[3])
   r,t,d,x,y = self:face_to_texture_transform_and_dimension(fid, true)
   printf(" -- rot: %2.4f %2.4f %2.4f %2.4f",r[1],r[2],r[3],r[4])
   print(self.model.face_verts[fid])
   printf(" -- trs: %2.4f %2.4f %2.4f",t[1],t[2],t[3])
   printf(" -- dim: %d %d %d",d[1],d[2],d[3])
   printf(" -- xrg: %2.4f - %2.4f",x[1],x[2])
   printf(" -- yrg: %2.4f - %2.4f",y[1],y[2])
end

-- + ----------------
-- compute_alpha(dir,d,norm)
--  prefer smaller angle w/ respect to norm
-- (normalize with pi/2 as that is the biggest angle)
function TextureBuilder:compute_alpha(dir,d,norm,debug)
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

function TextureBuilder:range_to_texture_dimensions(xrange, yrange, debug)
   local widthpx  = math.max(2,math.floor(xrange[3] * self.ppm + 1))
   local heightpx = math.max(2,math.floor(yrange[3] * self.ppm + 1))
   local linx = torch.linspace(xrange[1],xrange[2],widthpx)
   local liny = torch.linspace(yrange[1],yrange[2],heightpx)
   local dx = xrange[3]/widthpx
   local dy = yrange[3]/heightpx
   return widthpx, heightpx, linx, liny, dx, dy
end

function TextureBuilder:xyz_plane(fid, debug)
  local model       = self.model
  local scale_inv   = self.scale_inv
  local views       = model.views

  -- 1) find rotation and translation to texture coords and dimensions of texture
  local rot,trans,dims,xrange,yrange = 
     self:face_to_texture_transform_and_dimension(fid, debug)

  -- a) need rotation from texture to global
  local rotT = geom.quaternion.conjugate(rot)

  -- 2) create the texture image which we will color, and temp variables

  local widthpx, heightpx, linx, liny, dx, dy = self:range_to_texture_dimensions(xrange, yrange, debug)

  local v      = torch.ones(heightpx,widthpx,3)

  v:select(3,dims[1]):fill(0)
  v:select(3,dims[2]):copy(linx:reshape(1,widthpx):expand(heightpx,widthpx))
  v:select(3,dims[3]):copy(liny:reshape(heightpx,1):expand(heightpx,widthpx))

  -- points are now in global coordinates
  local xyz = geom.quaternion.rotate_translate(rotT,trans,v)
  return xyz
end

-- produce list of points in xyz for painting edges of a face at a
-- certain pixels per meter.
function TextureBuilder:xyz_wireframe(fid,ppm) 
  local model      = self.model

  local face_verts = model.face_verts[fid] 
  local n_verts    = model.n_verts_per_face[fid]

  ppm = ppm or 100   -- pixels per meter

  -- don't need homogeneous coords here
  local f = face_verts:narrow(1,1,n_verts):narrow(2,1,3)
  local directions = torch.Tensor(n_verts, 3)
  local pvert      = f[n_verts]

  -- compute directions
  for vi = 1,n_verts do
     local cvert = f[vi]
     directions[vi]:copy(cvert - pvert)
     pvert = cvert
  end

  local lengths = directions:norm(2,2)
  local n_px    = torch.mul(lengths,ppm):ceil()
  local steps   = torch.cdiv(directions,n_px:expand(directions:size()))

  local total_pts = n_px:sum()
  local pts       = torch.Tensor(total_pts, 3)
  local pi        = 1
  prev  = f[n_verts]
  for vi = 1,n_verts do
     local step = steps[vi]
     local nptx = n_px[vi]:squeeze() 
     for s = 1,nptx do
        pts[pi] = prev + step     
        prev    = pts[pi]
        pi      = pi + 1
     end
  end
  return pts
end

-- get file path for a face's texture
function TextureBuilder:file(fid)
  local name = paths.basename(self.scan.model_file):gsub(".obj$", "_face-"..fid..".png")
  return paths.concat(self.output_dir, name)
end

-- save the texture for a face and update the materials and submeshes
function TextureBuilder:save_img(fid)
  local texture_img = self.textures[fid]
  local material = {
    name = "face"..fid,
    diffuse = {0.5, 0.5, 0.5, 1}
  }
  if texture_img then
    local img_file = self:file(fid)
    win = image.display{image=texture_img,min=0,max=1,win=win}
    image.save(img_file, texture_img)
    if paths.filep(img_file) then
      log.trace('Texture saved for face', fid, img_file)
      material.diffuse_tex_path = img_file
    else
      log.trace('No texture saved for face', fid)
    end
  else
    log.trace('No texture exists for face', fid)
  end
  table.insert(self.model.materials, material)
  table.insert(self.model.submeshes, {fid, fid, #self.model.materials})
end

