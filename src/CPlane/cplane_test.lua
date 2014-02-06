cplane_ffi = require('./cplane_ffi')

pc = PointCloud.PointCloud.new('/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/010/sweep.xyz')
normals = pc:get_normal_map()
points = pc:get_xyz_map()
res = torch.Tensor(points:size(2), points:size(3))
errors = torch.Tensor(points:size(2), points:size(3))
normals_ret = torch.Tensor(normals:size())


--t = torch.Tensor(3,20,20):fill(2)
--res = torch.Tensor(20,20)

--t = torch.rand(3,3,3)
--res = torch.Tensor(3,3)
window = 13
dist_thresh = 81.0
plane_thresh = 0.025


timer = torch.Timer.new()

t0 = timer:time()['real']
cplane_ffi.classifyPoints( torch.cdata(points), window, dist_thresh, plane_thresh, torch.cdata(res), torch.cdata(errors), torch.cdata(normals_ret) ) 
print( "dt: ", timer:time()['real'] - t0 )

--cplane_ffi.classifyPoints( torch.cdata(t), window, plane_thresh, torch.cdata(res) ) 
print(res)

print(errors:max())
print(errors:min())

image.display(normals)
image.display(res)
image.display(errors)
image.display(points)
image.display(normals_ret)
