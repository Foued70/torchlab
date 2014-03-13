--[[
	Theta remapping test
		since each scan has a different theta and I want to project masks from one scan onto another
		then it would be nice to have a index mapping from one scan to another
]]--

ArcIO = data.ArcIO
arc_io = ArcIO.new('precise-transit-6548', '')

pc1 = arc_io:getScan(1)
pc2 = arc_io:getScan(2)

_,_,theta1 = pc1:get_xyz_map()
_,_,theta2 = pc2:get_xyz_map()

print(theta1[{{1},{}}])
print(theta2[{{1},{}}])

sub_thetas = theta1[{{1},{}}]:squeeze()
base_thetas = theta2[{{1},{}}]:squeeze()

-- The search bound should at least be bigger than the difference between 
-- 	 the theta sizes 
search_bound = 10
print("search_bound: ", search_bound)

ind_map = torch.LongTensor(sub_thetas:nElement())

j = 1
for i = 1,sub_thetas:nElement() do 
	-- Search for closest element from t2 to t1 around j 
	local min_d = nil
	local min_j = nil
	for k = j-search_bound,j+search_bound do 
		if k > 1 and k < base_thetas:nElement() then
			cur_d = torch.pow(sub_thetas[i] - base_thetas[k],2) 
			if not min_d or cur_d < min_d then
				min_d = cur_d
				min_j = k
			end
		end
	end
	-- Set j to closest element 
	j = min_j
	ind_map[i] = j 
	print(string.format("[%d,%d]:[%f, %f]", i, j, sub_thetas[i], base_thetas[j]))
end

-- Use :index function to create indices lists mapping from bigger to smaller, and sub to map from 
-- smaller to bigger 
base_map = torch.reshape( torch.range(1,theta2:size(1)*theta2:size(2)), theta2:size(1), theta2:size(2) ) 
print(ind_map)
print(base_map)
sub_map = base_map:index(2, ind_map)
print(sub_map)



