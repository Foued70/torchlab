--[[ 
	Naive segmentation attempt via thresholding of normals and plane-to-plane distances
]]-- 

-- Vectorized cosine distance function
-- Give it two 3xnxm matrices and it will compute the cosine distance between each element
-- Returns: cosine_distance, mask where den==0 
-- Need to check into numerical problems 
function cosine_distance_vec( A, B ) 
	-- Compute dot product 
	-- num = torch.cmul(A,B):sum(1) Not sure why sum(1) isn't working
	local Z = torch.cmul(A,B) 
	local num = Z[{1,{},{}}] + Z[{2,{},{}}] + Z[{3,{},{}}] 
	-- Compute magnitude 
	-- den = torch.cmul(A:pow(2):sum(1),B:pow(2):sum(1))
	local Apow = A:pow(2)
	local Bpow = B:pow(2)
	local den = torch.cmul((Apow[{1,{},{}}] + Apow[{2,{},{}}] + Apow[{3,{},{}}]):sqrt(), 
						   (Bpow[{1,{},{}}] + Bpow[{2,{},{}}] + Bpow[{3,{},{}}]):sqrt())

	-- Mask for 0 denominator values
	local div0_mask = torch.ByteTensor( den:size() )
	div0_mask[ den:lt(0) ] = 1

	-- Now set those zeros to 1 so that we don't have nan values 	
	den[ den:eq(0) ] = 1
	
	return torch.acos( torch.cdiv(num, den) ), div0_mask
end

-- Job 
job_id = "precise-transit-6548"

-- Saved data output directory 
output_dir = '/Users/uriah/Downloads/' .. job_id .. '/work/output/segmentation/'

scan_start = 22
scan_end = 22
for scan_num = scan_start, scan_end do
	print("Outputing Points for Segmented Region in scan_num: ", scan_num)

	scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)

	-- Load in pointcloud
	pc = PointCloud.PointCloud.new( scan_fname )

	-- Vecotorized cosine distance in x-dir  ... TODO: better naming 
	normal_map,dd,phi,theta,pc_mask = pc:get_normal_map()
	x0 = normal_map[{{},{},{1,normal_map:size(3)-1}}]
	x1 = normal_map[{{},{},{2,normal_map:size(3)}}]

	y0 = normal_map[{{},{1,normal_map:size(2)-1},{}}]
	y1 = normal_map[{{},{2,normal_map:size(2)},{}}]
	
	x_cosine_distance, x_div0_mask = cosine_distance_vec( x0, x1 )
	y_cosine_distance, y_div0_mask = cosine_distance_vec( y0, y1 )

	print(x_cosine_distance:size())
	print(y_cosine_distance:size())

	x = x_cosine_distance[{{2,normal_map:size(2)},{}}]
	y = y_cosine_distance[{{}, {2,normal_map:size(3)}}]

	print(x_cosine_distance:min())
	print(y_cosine_distance:min())
	print(x_cosine_distance:max())
	print(y_cosine_distance:max())

	image.display(x_cosine_distance)
	image.display(y_cosine_distance)

	non_smooth_mask = torch.Tensor(x:size()):fill(0)
	thresh = -0.1 -- Thesh isn't acting correctly, seems like numerical problems in cosine_distance_vec, DEBUG
	non_smooth_mask[ x:ge(thresh) ] = 1
	non_smooth_mask[ y:ge(thresh) ] = 1
	image.display( non_smooth_mask )
end

