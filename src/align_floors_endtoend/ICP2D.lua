ICP2D = Class()

local function findBestTransformation(points1, points2) 
	local leftSide = torch.zeros(2*points1:size(1), 4)
	local rightSide = torch.zeros(2*points1:size(1), 1)

	for i=1, points1:size(1) do
		leftSide[{2*i-1,{}}] = torch.Tensor({points1[i][1], -points1[i][2], 1, 0})
		leftSide[{2*i,{}}] = torch.Tensor({points1[i][2], points1[i][1], 0, 1})
		rightSide[{2*i-1,{}}] = torch.Tensor({points2[i][1]})
		rightSide[{2*i,{}}] = torch.Tensor({points2[i][2]})

	end

	local transf = torch.inverse(leftSide:t()*leftSide)*(leftSide:t()*rightSide):reshape(4)
	local H = torch.Tensor({transf[1], -transf[2], transf[3], transf[2], transf[1], transf[4], 0,0,1}):reshape(3,3)
	return geom.Homography.new(H)

end
local function findBestTransformation3D(points1, points2) 
	local leftSide = torch.zeros(2*points1:size(1), 12)
	local rightSide = torch.zeros(2*points1:size(1), 1)

	for i=1, points1:size(1) do
		leftSide[{2*i-1,{}}] = torch.Tensor({points1[i][1], -points1[i][2], 1, 0})
		leftSide[{2*i,{}}] = torch.Tensor({points1[i][2], points1[i][1], 0, 1})
		rightSide[{2*i-1,{}}] = torch.Tensor({points2[i][1]})
		rightSide[{2*i,{}}] = torch.Tensor({points2[i][2]})

	end

	local transf = torch.inverse(leftSide:t()*leftSide)*(leftSide:t()*rightSide):reshape(4)
	local H = torch.Tensor({transf[1], -transf[2], transf[3], transf[2], transf[1], transf[4], 0,0,1}):reshape(3,3)
	return geom.Homography.new(H)

end
function ICP2D.align(points1, points2, max_iterations, max_distance)
	if(not(max_iterations)) then
		max_iterations = 50
	end
	if not(max_distance) then
		max_distance = 2
	end
	local iterations = 0
	local score = 0
	local H
	if(points1:size(2) == 2) then
		H = geom.Homography.new(torch.eye(3))
	else
		H = geom.Homography.new(torch.eye(4))
	end
	while(iterations<max_iterations) do
		local ind, dis = opencv.flann.flann_knn(opencv.Mat.new(points1), opencv.Mat.new(points2), 1)
		ind = ind:toTensor():long()
		dis = dis:toTensor()
		local score_n = torch.lt(dis,max_distance):sum()
		if (score_n > score) then
			score = score_n
		else 
			break
		end
		local t = image.thresholdReturnCoordinates(dis,max_distance, true):long()
		local points1_t = points1:index(1,t)
		local points2_t = points2:index(1,ind:index(1,t):reshape(t:size(1)))
		if(points1:size(2) == 2) then
			H=findBestTransformation(points1_t, points2_t)
		else
			H=findBestTransformation3D(points1_t, points2_t)
		end
		points1 = H:applyToPointsReturn2d(points1:t()):t()
		iterations = iterations+1
	end
	return H
end
