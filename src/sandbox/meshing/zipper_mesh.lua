path = require 'path'

fname    = 'data/297.xyz'
img_name = '297.png'

_G.p = PointCloud.PointCloud.new('point.od') -- fname)

_G.xyz  = p:get_xyz_map()

n_cells = xyz[1]:nElement()
n_verts = p.count 

height = xyz:size(2)
width  = xyz:size(3)

-- mask tells us where the faro returns no points
_G.mask_map = p.mask_map:clone()

-- print(nverts ==  mask_map:nElement() - mask_map:sum())

-- vertex_map: map_index to point_index
-- vertex_list: point_index to map_index
vertex_map  = torch.LongTensor():zeros(n_cells)
vertex_list = torch.LongTensor(n_verts)
mask_flat   = mask_map:reshape(n_cells)

vert_idx    = 1
for i = 1,n_cells do 
   if mask_flat[i] == 0 then 
      vertex_map[i] = vert_idx
      vertex_list[vert_idx] = i
      vert_idx = vert_idx + 1
   end
end

uvs_temp = torch.FloatTensor(2,n_cells)

x = torch.linspace(0,1,width):resize(1,width):expand(height,width)
y = torch.linspace(-1,0,height):mul(-1):resize(1,height):expand(width,height)

uvs_temp[1]:copy(x)
uvs_temp[2]:copy(y:t())

_G.uvs = torch.FloatTensor(2,n_verts)
uvs[1]:copy(uvs_temp[1][vertex_list])
uvs[2]:copy(uvs_temp[2][vertex_list])

_G.uvs = uvs:transpose(1,2):contiguous()

_G.faces = torch.LongTensor():resize(xyz:size())

faces[1]:copy(torch.range(1,n_cells)) -- self

-- over 
faces[2]:copy(faces[1]):add(1)
faces[{2,{},width}]:fill(1) -- invalid right most row

-- down
faces[3]:copy(faces[1]):add(width)
faces[{3,height,{}}]:fill(1) -- invalid bottom row

faces:resize(3,n_cells)

-- whack bottom and far right wrap around 
mask_map[{{},width}]  = 1
mask_map[{height,{}}] = 1

mask_flat = mask_map:reshape(n_cells)

-- want to mask all faces which touch a non-existing vertex. At this point faces uses the map_index
_G.mm   = mask_flat[faces[1]] + mask_flat[faces[2]] + mask_flat[faces[3]]
_G.mm   = mm:eq(0) -- invert this is a map of good faces

n_good_faces = mm:sum()

fid = 1

-- index to only the good faces
face_index = torch.LongTensor(n_good_faces)
for i = 1,mm:size(1) do 
   if mm[i] > 0 then 
      face_index[fid] = i
      fid = fid + 1
   end
end

-- print(fid - n_good_faces .. " should == 1")

_G.faces_all = torch.LongTensor():zeros(3,3,n_good_faces)
faces_clean = faces_all[2]
faces_clean[1]:copy(vertex_map[faces[1][face_index]])
faces_clean[2]:copy(vertex_map[faces[3][face_index]])
faces_clean[3]:copy(vertex_map[faces[2][face_index]])

-- one uv per vertex (same id)
faces_all[3]:copy(faces_all[2])

_G.o      = data.Obj.new()
o.verts   = p.points
o.n_verts = p.count

o.uvs   = uvs
o.n_uvs = uvs:size(1)

o.faces = faces_all:transpose(1,3)
o.n_faces = n_good_faces
-- all tris
o.n_verts_per_face = torch.LongTensor(n_good_faces):fill(3)

material = {}
material.name = img_name
material.diffuse = {0.5, 0.5, 0.5, 1}
material.ambient = {0, 0, 0, 1}
material.diffuse_tex_path = img_name


o.materials = { material }


_G.objname = path.basename(fname):gsub('xyz','obj')

o:save(objname)
