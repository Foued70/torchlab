local pcl = PointCloud.pointcloud

local filters={}

function filters.radial_filter(ptcld, radius, min_neighbors)
	if not ptcld.k3dtree then
		ptcld:make_3dtree()
	end
	local index_table = {}
	for i = 1,ptcld.count do
		local i,d,s = ptcld.k3dtree:radius_search(ptcld.points[i], radius, min_neighbors)
		if s >= min_neighbors then
			table.insert(index_table, i)
		end
	end
	local index_tensor = torch.Tensor(index_table)
	return ptcld:make_sub_pointcloud(index_tensor)
end

return filters