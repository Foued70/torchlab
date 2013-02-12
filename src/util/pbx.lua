local torch = require "torch"
local pb = require "protobuf"
local geom = require "util/geom"


local function clean_color(color)
  if not color then return nil end
  return {r = color.r, g = color.g, b = color.b, a = color.a}
end


local function clean_materials(materials)
  mats = {}
  for i, m in ipairs(materials) do
    mats[i] = {
      name = m.name,
      ambient = clean_color(m.ambient),
      diffuse = clean_color(m.diffuse),
      specular = clean_color(m.specular),
      shininess = m.shininess,
      alpha = m.alpha,
      illumType = m.illumType,
      diffuseTexPath = m.diffuseTexPath
    }
  end

  return mats
end


local function parse(pbx_data)
  local model_data = pb.ModelData()
	model_data:ParseFromString(pbx_data)


  local materials = model_data.materials
  local material_map = {}
  for mat_i, mat in ipairs(materials) do
    material_map[mat.name] = mat_i
  end

  local n_verts = 0
  local n_faces = 0
  local n_submeshes = 0
  for j, mesh in ipairs(model_data.meshes) do
    n_verts = n_verts + #mesh.vertices
    n_submeshes = n_submeshes + #mesh.submeshes

    for k, submesh in ipairs(mesh.submeshes) do
      n_faces = n_faces + (#submesh.triangles / 3)
    end
  end

  local verts = torch.Tensor(n_verts,4):fill(1)
  local uvs = torch.Tensor(n_verts,2)
  local faces = torch.IntTensor(n_faces * 3)
  local submesh_ranges = torch.IntTensor(n_submeshes,2)
  local submesh_materials = torch.IntTensor(n_submeshes)

  local vert_i = 1
  local face_i = 1
  local submesh_i = 1

  for j, mesh in ipairs(model_data.meshes) do
    for k, submesh in ipairs(mesh.submeshes) do
      local triangles = submesh.triangles

      -- all vert arrays will be merged, so adjust the indexes
      -- this also takes care of the 0/1 index issue
      tri_vert_indexes = torch.Tensor(triangles)
      tri_vert_indexes:add(vert_i)

      face_end_i = face_i + #triangles - 1
      faces[{{ face_i, face_end_i }}] = tri_vert_indexes

      submesh_ranges[{submesh_i, 1}] = face_i
      submesh_ranges[{submesh_i, 2}] = face_end_i + 1
      submesh_materials[submesh_i] = material_map[submesh.materialName]

      face_i = face_end_i + 1
      submesh_i = submesh_i + 1
    end

    local mesh_uvs = mesh.uvs
    for k, vert in ipairs(mesh.vertices) do
      verts[vert_i][1] = vert.x
      verts[vert_i][2] = vert.y
      verts[vert_i][3] = vert.z

      uvs[vert_i][1] = mesh_uvs[k].x
      uvs[vert_i][2] = mesh_uvs[k].y

      vert_i = vert_i + 1
    end

  end


  faces:resize(n_faces, 3)

  local face_verts    = torch.Tensor(n_faces, 3, 3)
  local face_normals  = torch.Tensor(n_faces,3)
  local d             = torch.Tensor(n_faces) 
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

    -- b) compute object centers
    centers[fid] = fverts:mean(1):squeeze()

    -- c) compute plane normal and d distance from origin for plane eq.
    face_normals[fid] = geom.compute_normal(fverts)
    d[fid]       = - torch.dot(face_normals[fid],centers[fid])

    -- d) compute bbox
    local thisbb   = bbox[fid]
    thisbb:narrow(1,1,3):copy(fverts:min(1):squeeze())
    thisbb:narrow(1,4,3):copy(fverts:max(1):squeeze())
    
  end

  local obj = {}
  obj.materials          = clean_materials(materials)
  obj.verts              = verts
  obj.uvs                = uvs
  obj.faces              = faces
  obj.face_verts         = face_verts
  obj.face_normals       = face_normals
  obj.face_center_dists  = d
  obj.face_centers       = centers
  obj.bbox               = bbox
  obj.submesh_ranges     = submesh_ranges
  obj.submesh_materials  = submesh_materials


  return obj
end

local function load(filename)
  io.input(filename)
  return parse(io.read("*all"))
end

local exports = {}

exports.load = load
exports.parse = parse

return exports
