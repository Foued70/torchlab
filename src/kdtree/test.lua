libflann=require './libflann'
ctorch=util.ctorch
dname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/dset.xyz'
tname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/tset.xyz'
oname='/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/output.txt'
dset= libflann.read_xyz_file(dname)
tset= libflann.read_xyz_file(tname)

nn = 15
flann_params=libflann.new_flann_params_pointer({algorithm=libflann.clib.FLANN_INDEX_KDTREE,trees=10})

--indices,dists = libflann.find_nearest_neighbors(dset, tset, nn, flann_params)

index,z = libflann.build_index(dset, flann_params)

indices, dists = libflann.find_nearest_neighbors_index(index, tset, nn, flann_params)

k=1
--indices,dists,nn = libflann.radius_search(index, tset[k], nn, 0.5, flann_params)

i=1
for i = 1,tset:size(1) do
	print("POINT: {"..tset[i+k-1][1]..", "..tset[i+k-1][2]..", "..tset[i+k-1][3].."}")
	for j=1,nn do
		local ind = indices[(i-1)*nn+j]
		local dis = dists[(i-1)*nn+j]
		if ind >= 0 then
			print("row: "..ind..", dist: "..dis.." : ("..dset[ind+1][1]..", "..dset[ind+1][2]..", "..dset[ind+1][3]..")")
		end
	end
	print("")
end

libflann.free_index(index,flann_params)