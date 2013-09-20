path = require 'path'

max_dist_btwn_pts = 10 -- 0.25 -- 10 cm

fname    = 'data/297.xyz'
img_name = '297.png'

_G.p = PointCloud.PointCloud.new('point.od') -- fname)

_G.xyz_map   = p:get_xyz_map()

map_height = xyz_map:size(2)
map_width  = xyz_map:size(3)
n_map_cells  = map_height * map_width

_G.xyz_map = xyz_map:reshape(3,n_map_cells)
grid_height = 128
grid_width  = 256
n_grid_cells = grid_height * grid_width

_G.grid_index = torch.linspace(1,map_width,grid_width):resize(1,grid_width):expand(grid_height,grid_width):long()
grid_index:add(torch.linspace(0,map_height-1,grid_height):resize(1,grid_height):expand(grid_width,grid_height):t():long():mul(map_width))
grid_index:resize(grid_index:nElement())

print('grid_index', grid_index:max(), grid_index:min())
_G.grid_xyz_map = torch.Tensor(3,n_grid_cells)

grid_xyz_map[1]:copy(xyz_map[1][grid_index])
grid_xyz_map[2]:copy(xyz_map[2][grid_index])
grid_xyz_map[3]:copy(xyz_map[3][grid_index])
grid_xyz_map:resize(3,grid_height,grid_width)

n_verts = p.count 


-- mask tells us where the faro returns no points
_G.mask_map = p.mask_map:clone()

-- print(nverts ==  mask_map:nElement() - mask_map:sum())

-- vertex_map: map_index to point_index
-- vertex_list: point_index to map_index
vertex_map  = torch.LongTensor(n_map_cells):fill(-1)
vertex_list = torch.LongTensor(n_verts)
mask_flat   = mask_map:reshape(n_map_cells)

vert_idx    = 1
for i = 1,n_map_cells do 
   if mask_flat[i] == 0 then 
      vertex_map[i] = vert_idx
      vertex_list[vert_idx] = i
      vert_idx = vert_idx + 1
   end
end

grid_mask_map = mask_flat[grid_index]:reshape(grid_height,grid_width)
 
grid_vertex_map = vertex_map:reshape(vertex_map:nElement())[grid_index]

-- uvs belong to the points
uvs_temp = torch.FloatTensor(2,n_map_cells)

x = torch.linspace(0,1,map_width):resize(1,map_width):expand(map_height,map_width)
y = torch.linspace(-1,0,map_height):abs():resize(1,map_height):expand(map_width,map_height)

uvs_temp[1]:copy(x)
uvs_temp[2]:copy(y:t())

_G.uvs = torch.FloatTensor(2,n_verts)
uvs[1]:copy(uvs_temp[1][vertex_list])
uvs[2]:copy(uvs_temp[2][vertex_list])

_G.uvs = uvs:transpose(1,2):contiguous()


-- faces: self, up, over, down, back
_G.faces = torch.LongTensor():resize(5,grid_height,grid_width)

-- ## over/down faces
faces[1]:copy(torch.range(1,n_grid_cells)) -- self

-- up
faces[2]:copy(faces[1]):add(-grid_width)
faces[{2,1,{}}]:fill(1) -- invalid top row

-- back
faces[3]:copy(faces[1]):add(-1)
faces[{3,{},1}]:fill(1) -- invalid left most row

-- down
faces[4]:copy(faces[1]):add(grid_width)
faces[{4,grid_height,{}}]:fill(1) -- invalid bottom row

-- over 
faces[5]:copy(faces[1]):add(1)
faces[{5,{},grid_width}]:fill(1) -- invalid right most row

faces:resize(5,n_grid_cells)

faces_all = torch.LongTensor():zeros(4,3,n_grid_cells)

faces_up_back = faces_all[1]
faces_up_back[1]:copy(faces[1])   -- self
faces_up_back[2]:copy(faces[2])   -- up
faces_up_back[3]:copy(faces[3])   -- back

faces_back_down = faces_all[2]
faces_back_down[1]:copy(faces[1]) -- self
faces_back_down[2]:copy(faces[3]) -- back
faces_back_down[3]:copy(faces[4]) -- down

faces_down_over = faces_all[3]
faces_down_over[1]:copy(faces[1]) -- self
faces_down_over[2]:copy(faces[4]) -- down
faces_down_over[3]:copy(faces[5]) -- over

faces_over_up = faces_all[4]
faces_over_up[1]:copy(faces[1])   -- self
faces_over_up[2]:copy(faces[5])   -- over
faces_over_up[3]:copy(faces[2])   -- up


-- whack bottom and far right wrap around 
grid_mask_map[{{},grid_width}]  = 1
grid_mask_map[{grid_height,{}}] = 1

-- remove excessive depth
dmask = torch.ByteTensor(4,grid_height,grid_width)
diffm = torch.FloatTensor(grid_height,grid_width+1)

-- over/back
d = grid_xyz_map[{{},{},{1,grid_width-1}}] - grid_xyz_map[{{},{},{2,grid_width}}]
diffm[{{},{2,grid_width}}] = d:cmul(d):sum(1):squeeze():sqrt()

diffm[{{},1}]            = 1
diffm[{{},grid_width+1}] = 1

dmask[2] = diffm[{{},{1,grid_width}}]:gt(max_dist_btwn_pts)   -- back
dmask[4] = diffm[{{},{2,grid_width+1}}]:gt(max_dist_btwn_pts) -- over

diffm:resize(grid_height+1,grid_width):zero()

-- up/down
d = grid_xyz_map[{{},{1,grid_height-1},{}}] - grid_xyz_map[{{},{2,grid_height},{}}]
diffm[{{2,grid_height},{}}] = d:cmul(d):sum(1):squeeze():sqrt()

diffm[{1,{}}]        = 1
diffm[{grid_height+1,{}}] = 1

dmask[1] = diffm[{{1,grid_height},{}}]:gt(max_dist_btwn_pts)   -- up
dmask[3] = diffm[{{2,grid_height+1},{}}]:gt(max_dist_btwn_pts) -- down

diffm = nil
collectgarbage()

-- default mask must zap every face for which there is no vertex at self
_G.face_mask = grid_mask_map:repeatTensor(4,1,1)
face_mask[1] = face_mask[1] + dmask[1] + dmask[2] -- self, up, back
face_mask[2] = face_mask[2] + dmask[2] + dmask[3] -- self, back, down
face_mask[3] = face_mask[3] + dmask[3] + dmask[4] -- self, down, over
face_mask[4] = face_mask[4] + dmask[4] + dmask[1] -- self, over, up

_G.face_mask   = face_mask:eq(0) -- invert this is a map of good faces

_G.face_mask = face_mask:reshape(4,n_grid_cells):transpose(2,1):contiguous():reshape(face_mask:nElement())
_G.faces = faces_all:transpose(1,2):transpose(2,3):contiguous():reshape(3,n_grid_cells*4)
print(faces)
print(face_mask)
print(grid_vertex_map)
faces_all = nil
dmask = nil
collectgarbage()

n_good_faces = face_mask:sum()

fid = 1

-- index to only the good faces
face_index = torch.LongTensor(n_good_faces)
for i = 1,face_mask:size(1) do 
   if face_mask[i] > 0 then 
      face_index[fid] = i
      fid = fid + 1
   end
end

print(fid - n_good_faces .. " should == 1")

_G.faces_all = torch.LongTensor():zeros(3,3,n_good_faces)
faces_clean = faces_all[2]
faces_clean[1]:copy(grid_vertex_map[faces[1][face_index]])
faces_clean[2]:copy(grid_vertex_map[faces[2][face_index]])
faces_clean[3]:copy(grid_vertex_map[faces[3][face_index]])

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


_G.objname = path.basename(fname):gsub('.xyz','-subsample-' .. max_dist_btwn_pts .. '.obj')

o:save(objname)
