libflann=require './libflann'
ctorch=util.ctorch
dname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/dset.xyz'
tname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/tset.xyz'
oname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/output.txt'
dset= libflann.read_xyz_file(dname)
tset= libflann.read_xyz_file(tname)

nn = 5
flann_params=libflann.new_flann_params_pointer({algorithm=libflann.clib.FLANN_INDEX_KDTREE,trees=10})

--indices,dists = libflann.find_nearest_neighbors(dset, tset, nn, flann_params)

index,z = libflann.build_index(dset, flann_params)

--indices, dists = libflann.find_nearest_neighbors_index(index, tset, nn, flann_params)

indices,dists = libflann.radius_search(index, tset[1], nn, 0.025, flann_params)

--for i = 1,tset:size(1) do
for i = 1,1 do
	print("POINT: {"..tset[i][1]..", "..tset[i][2]..", "..tset[i][3].."}")
	for j=1,nn do
		local ind = indices[(i-1)*nn+j]
		local dis = dists[(i-1)*nn+j]
		print("row: "..ind..", dist: "..dis.." : ("..dset[i+1][1]..", "..dset[i+1][2]..", "..dset[i+1][3]..")")
	end
	print("")
end

libflann.free_index(index,flann_params)