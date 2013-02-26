local ObjectData = torch.class('ObjectData')

require 'image' -- fucking image global
local geom = require 'util.geom'

function ObjectData:__init(filename)
  if filename then
    self:load(filename)
    self:add_derived_data()
  end
end

local _t = sys.clock()
local function tic(msg)
  local lapse = sys.clock() - _t
  _t = sys.clock()
  if msg then
    print(lapse, msg)
  end
end

local function get_counts(filename)
  local n_faces = 0;
  local n_tmp_verts = 0;
  local n_tmp_uvs = 0;
  for line in io.lines(filename) do
    if line:match('^f ') then
      n_faces = n_faces + 1
    elseif line:match('^v ') then
      n_tmp_verts = n_tmp_verts + 1
    elseif line:match('^vt ') then
      n_tmp_uvs = n_tmp_uvs + 1;
    end
  end

  return n_faces, n_tmp_verts, n_tmp_uvs
end


function ObjectData:add_derived_data()
  tic()
  local verts = self.verts
  local faces = self.faces
  local n_faces = faces:size(1)

  local face_verts    = torch.Tensor(n_faces, 3, 3)
  local face_normals  = torch.Tensor(n_faces,3)
  local face_center_dists             = torch.Tensor(n_faces)
  local centers       = torch.Tensor(n_faces,3)
  local bbox          = torch.Tensor(n_faces,6) -- xmin,ymin,zmin,xmax,ymax,zmax

  for fid = 1,n_faces do
    local nverts = 3
    local fverts = face_verts[fid]:narrow(1,1,nverts)
    local face   = faces[fid]

    -- a) build face_verts (copy all the data)
    for j = 1,nverts do
      fverts[j] = verts[ {face[j],{1,3}} ]
    end

    -- b) compute face centers
    centers[fid] = fverts:mean(1):squeeze()

    -- c) compute plane normal and face_center_dists distance from origin for plane eq.
    face_normals[fid] = geom.compute_normal(fverts)
    face_center_dists[fid]       = - torch.dot(face_normals[fid],centers[fid])

    -- face_center_dists) compute bbox
    local thisbb   = bbox[fid]
    thisbb:narrow(1,1,3):copy(fverts:min(1):squeeze())
    thisbb:narrow(1,4,3):copy(fverts:max(1):squeeze())

  end

  self.face_verts         = face_verts
  self.face_normals       = face_normals
  self.face_center_dists  = face_center_dists
  self.face_centers       = centers
  self.bbox               = bbox

  tic('derive')
end

local function parse_color(line)
  local r, g, b = line:match('[^%s]+%s+([^%s]+)%s+([^%s]+)%s+([^%s]+)')
  return {tonumber(r), tonumber(g), tonumber(b), 1}
end

local function trim(s)
  return s:match'^%s*(.*%S)' or ''
end

local function load_materials(pathname, filename)
  filename = paths.concat(pathname, filename)
  local materials = {}
  local mtl_name_index_map = {}
  local mtl

  for line in io.lines(filename) do
    if line:match('^newmtl ') then
      mtl = {name = trim(line:sub(8))}
      table.insert(materials, mtl)

      mtl_name_index_map[mtl.name] = #materials
    elseif line:match('^Ns ') then
      mtl.shininess = tonumber(line:sub(4))
    elseif line:match('^Ka ') then
      mtl.ambient = parse_color(line)
    elseif line:match('^Kd ') then
      mtl.diffuse = parse_color(line)
    elseif line:match('^Ks ') then
      mtl.specular = parse_color(line)
    elseif line:match('^d ')  then
      mtl.alpha = tonumber(line:sub(3))
    elseif line:match('^Tr ') then
      mtl.alpha = tonumber(line:sub(4))
    elseif line:match('^illum ') then
      mtl.illumType = tonumber(line:sub(7))
    elseif line:match('^map_Kd ') then
      mtl.diffuse_tex_path = paths.concat(pathname, trim(line:sub(8)))
      mtl.diffuse_tex_img = image.load(mtl.diffuse_tex_path, nil, 'byte')
    end
  end

  return materials, mtl_name_index_map
end



function ObjectData:load(filename)
  tic()
  local n_faces, n_tmp_verts, n_tmp_uvs = get_counts(filename);
  tic('counts')

  local tmp_verts = torch.Tensor(n_tmp_verts,3)
  local tmp_uvs = torch.Tensor(n_tmp_uvs,2)
  local faces = torch.IntTensor(n_faces,3)
  local verts = torch.Tensor(n_faces*3,9):fill(1)

  tic('alloc')

  local materials, mtl_name_index_map
  local submeshes = {}

  local vert_cache = {}

  local tmp_verts_i = 1
  local tm_uv_i = 1
  local face_i = 1
  local next_vert_i = 1
  for line in io.lines(filename) do
    if line:match('^v ') then
      local x, y, z = line:match('%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)', 3)
      tmp_verts[{tmp_verts_i, 1}] = tonumber(x)
      tmp_verts[{tmp_verts_i, 2}] = tonumber(y)
      tmp_verts[{tmp_verts_i, 3}] = tonumber(z)
      tmp_verts_i = tmp_verts_i + 1
    elseif line:match('^vt ') then
      local u, v = line:match('%s*([^%s]+)%s*([^%s]+)', 4)
      tmp_uvs[{tm_uv_i, 1}] = tonumber(u)
      tmp_uvs[{tm_uv_i, 2}] = tonumber(v)
      tm_uv_i = tm_uv_i + 1;
    elseif line:match('^f ') then
      local vert_ids = {}
      vert_ids[1], vert_ids[2], vert_ids[3] = line:match('%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)', 3)
      for face_vert_i, vert_id in ipairs(vert_ids) do
        local vert_i = vert_cache[vert_id]
        if not vert_i then
          vert_i = next_vert_i

          local v_pos_i, v_uv_i = vert_id:match('(%d+)/(%d+)')
          v_pos_i, v_uv_i = tonumber(v_pos_i), tonumber(v_uv_i)

          verts[{vert_i, {1, 3}}] = tmp_verts[v_pos_i]
          verts[{vert_i, {5, 6}}] = tmp_uvs[v_uv_i]
          vert_cache[vert_id] = vert_i

          next_vert_i = next_vert_i + 1
        end

        faces[{face_i, face_vert_i}] = vert_i
      end

      face_i = face_i + 1;
    elseif line:match('^usemtl ') then
      local mtl_name = trim(line:sub(8))
      local mtl_id = mtl_name_index_map[mtl_name]

      if #submeshes > 0 then
        submeshes[#submeshes][2] = face_i - 1
      end

      table.insert(submeshes, {face_i, 0, mtl_id})
    elseif line:match('^mtllib ') then
      local mtllib = trim(line:sub(8))
      materials, mtl_name_index_map = load_materials(paths.dirname(filename), mtllib)
    end
  end

  if #submeshes > 0 then
    submeshes[#submeshes][2] = face_i - 1
  end

  tic('load tmp')

  n_verts = next_vert_i - 1
  local trim_verts = torch.Tensor(n_verts, 9)
  trim_verts[{{1,n_verts}}] = verts[{{1, n_verts}}]

  tic('trim')


  self.unified_verts      = trim_verts
  self.verts              = trim_verts:narrow(2, 1, 4)
  self.uvs                = trim_verts:narrow(2, 5, 2)
  self.faces              = faces
  self.submeshes          = torch.IntTensor(submeshes)
  self.materials          = materials
end



return ObjectData
