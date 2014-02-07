cplane_ffi = require('./cplane_ffi')

pc = PointCloud.PointCloud.new('/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/010/sweep.xyz')
normals = pc:get_normal_map()
points = pc:get_xyz_map()

timer = torch.Timer.new()

indices = torch.LongTensor( {1,1} )
covariances = torch.Tensor(9,1,1):fill(1)

t0 = timer:time()['real']
cplane_ffi.grow_region( torch.cdata(indices), torch.cdata(covariances) );
print( "dt: ", timer:time()['real'] - t0 )

