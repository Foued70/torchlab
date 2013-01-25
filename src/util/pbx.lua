local torch = require "torch"
local pb = require "protobuf"

local function load(filename)

	local model_data = pb.ModelData()
	io.input(filename); 
	model_data:ParseFromString(io.read("*all"));

  local n_verts = 0
  for mesh in model_data.meshes do
    n_verts = n_verts + #mesh.vertices
  end

  local verts = torch.Tensor(n_verts,4):fill(1)
  local i = 1
  for mesh in model_data.meshes do
    for vert in mesh.vertices do
      verts[i][1] = vert.x
      verts[i][2] = vert.y
      verts[i][3] = vert.z
      i = i + 1
    end
  end



  local obj = {}
  obj.nverts          = nverts
  obj.nfaces          = nfaces
  obj.verts           = verts
  obj.faces           = faces
  obj.nverts_per_face = nverts_per_face
  obj.face_verts      = face_verts
  obj.normals         = normals
  obj.d               = d
  obj.centers         = centers
  obj.bbox            = bbox

  return obj

end


local exports = {}

exports.load = load

return exports
