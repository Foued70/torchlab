cplane_ffi = require('./cplane_ffi')

pc = PointCloud.PointCloud.new('/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/010/sweep.xyz')
normals = pc:get_normal_map()
points = pc:get_xyz_map()
errors = torch.Tensor(points:size(2), points:size(3))
means_ret = torch.Tensor(normals:size())
normals_ret = torch.Tensor(normals:size())
second_moments_ret = torch.Tensor(9, normals:size(2), normals:size(3))
region_mask = torch.Tensor(points:size(2), points:size(3))
front_mask = torch.Tensor(points:size(2), points:size(3))

--t = torch.Tensor(3,20,20):fill(2)
--res = torch.Tensor(20,20)

--t = torch.rand(3,3,3)
--res = torch.Tensor(3,3)
window = 13
dist_thresh = 81.0
plane_thresh = 0.025


timer = torch.Timer.new()

t0 = timer:time()['real']
--cplane_ffi.classifyPoints( torch.cdata(points), window, dist_thresh, plane_thresh, torch.cdata(res), torch.cdata(errors), torch.cdata(normals_ret) ) 
cplane_ffi.classifyPoints( torch.cdata(points), window, dist_thresh, plane_thresh, torch.cdata(errors), torch.cdata(normals_ret), torch.cdata(means_ret), torch.cdata(second_moments_ret) ) 
print( "dt classify: ", timer:time()['real'] - t0 )


inds = torch.LongTensor( {300,200} )

t0 = timer:time()['real']
cplane_ffi.grow_plane_region( torch.cdata(inds), torch.cdata(normals_ret), torch.cdata(means_ret), torch.cdata(second_moments_ret), torch.cdata(region_mask), torch.cdata(front_mask) )
print( "dt region growing: ", timer:time()['real'] - t0 )

--cplane_ffi.classifyPoints( torch.cdata(t), window, plane_thresh, torch.cdata(res) ) 
image.display(normals)
image.display(errors)
image.display(normals_ret)
image.display(means_ret)
image.display( region_mask )
