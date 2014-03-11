_G.pv = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/plane_tools/plane_tools.lua"
_G.pk = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/pancakes.lua"
_G.tp = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/torch_plus.lua"
_G.planes = torch.load("/Users/joshua/Downloads/data/precise-transit-6548_003_1/top_planes.t7")
_G.pc = data.ArcIO.new("precise-transit-6548","test1"):getScan(3)

verbose = true
JT = torch.Timer()
print("Getting planes scores...")
_G.plane_stuff = pv.totally_visible_planes(planes,pc,verbose)
--_G.plane_stuff = torch.load("/Users/joshua/Downloads/data/precise-transit-6548_003_1/plane_stuff.t7")

print("Getting point-plane offsets...")
_G.plane_point_offsets = pv.verify_planes(planes,pc:get_xyz_map(),verbose)
max_point_offset = plane_point_offsets{1,#planes}:max()
min_point_offset = plane_point_offsets{1,#planes}:min()

print("Getting flat maps...")
_G.res = 10
for p = 1,#planes do
	if verbose then
		print("flat_map #",p)
	end
	depth_valid_mask = pc:get_valid_masks()
	--FLAT EDGE MAPS
	vEmap = tp.AND(plane_stuff[p].edge_mask, depth_valid_mask)
	if tp.ANY(vEmap) then
		Exyz_list = pc:get_xyz_list(vEmap) --also mask out depth invalid points
		t = torch.cdiv(-torch.Tensor(Exyz_list:size(1),1):fill(planes[p].eqn[4]),
						torch.mv(Exyz_list,planes[p].eqn[{{1,3}}])) -- t = -D/(N.x)  where N.x + D = 0
		tExyz_list = torch.cmul(Exyz_list,t:expandAs(Exyz_list))
		tEuv_list = pk.xyzToUV(tExyz_list,planes[p].eqn[{{1,3}}])

		--FLAT PLANE MAPS
		Puv_list = pk.xyzToUV(pc:get_xyz_list(planes[p].mask),planes[p].eqn[{{1,3}}]) --plane's uv list
		uv_list = torch.cat(tEuv_list,Puv_list,1)
		Pimg_ind = (Puv_list - uv_list:min(1):expandAs(Puv_list)) / res + 1
		Eimg_ind = (tEuv_list - uv_list:min(1):expandAs(tEuv_list)) / res + 1

		--FLAT OFFSET MAPS
		max_point_offset = torch.abs(plane_point_offsets(p)):max()
		Ovalues = ((torch.abs(plane_point_offsets(p)[planes[p].mask]))*255/max_point_offset):byte()

		flatP = pk.listToMap(uv_list,res,255)
		flatO = pk.listOnMap(Pimg_ind,torch.zeros(flatP:size()):byte(),Ovalues)
		flatE = pk.listOnMap(Eimg_ind,torch.zeros(flatP:size()):byte(),255)
		flatP:add(-flatE)
	else
		Puv_list = pk.xyzToUV(pc:get_xyz_list(planes[p].mask),planes[p].eqn[{{1,3}}]) --plane's uv list
		max_point_offset = torch.abs(plane_point_offsets(p)):max()
		Ovalues = ((torch.abs(plane_point_offsets(p)[planes[p].mask]))*255/max_point_offset):byte()

		flatO = pk.listToMap(Puv_list,res,Ovalues)
		flatP = pk.listToMap(Puv_list,res,255)
		flatE = torch.zeros(flatO:size()):byte()
	end
	flat = torch.ByteTensor(3,flatP:size(1),flatP:size(2)):fill(0)
	flat[1]:copy(flatO)
	flat[2]:copy(flatE)
	flat[3]:copy(flatP)
	plane_stuff[p].flat_map = flat:clone()
	plane_stuff[p].plane_mask = planes[p].mask:clone()
	plane_stuff[p].eqn = planes[p].eqn:clone()
end
print(JT:time().real," seconds")
