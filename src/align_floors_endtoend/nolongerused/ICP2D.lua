ICP2D = Class()

--[[Method from:
K. S. Arun, T. S. Huang,
and S. D. Blostein, "Least-Squares Fitting of Two 3-D Point Sets",
IEEE Transactions on Pattern Analysis and Machine Intelligence,
PAMI-9(5): 698 - 700, 1987.
To find least squares rotation and translation
]]--
local function findBestTransformation(points1, points2) 
	if points1:size(2) ~=2 or points2:size(2)~=2 then
		error("points size mus be N by 2")
	end
	local Xm = points1:mean(1)
	local Ym = points2:mean(1)
	local X1 = points1-torch.ones(points1:size(1),1)*Xm
	local Y1 = points2 - torch.ones(points2:size(1),1)*Ym
	local XtY = X1:t()*Y1
	local u,s,v = torch.svd(XtY)
	local R = u*v:t()
	print("here2")
	print(R)
	local T = Ym - Xm*R
	local H = torch.eye(3,3)
	H[{{1,2},{1,2}}] = R:t()
	H[{{1,2},3}] = T:t()
	print(H)
	return H


	--[[
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
	return H
	--]]

end
local function findBestTransformation3D(points1, points2) 
	error("not yet tested")
	local leftSide = torch.zeros(3*points1:size(1), 5)
	local rightSide = torch.zeros(3*points1:size(1), 1)

	for i=1, points1:size(1) do
		leftSide[{3*i-2,{}}] = torch.Tensor({points1[i][1], -points1[i][2], 1, 0,0})
		leftSide[{3*i-1,{}}] = torch.Tensor({points1[i][2], points1[i][1], 0, 1,0})
		leftSide[{3*i,{}}] = torch.Tensor({0, 0, 0, 0, 0,1+points2[i][3]})
		
		rightSide[{3*i-2,{}}] = torch.Tensor({points2[i][1]})
		rightSide[{3*i-1,{}}] = torch.Tensor({points2[i][2]})
		rightSide[{3*i,{}}] = torch.Tensor({points2[i][3]})
	end

	local transf = torch.inverse(leftSide:t()*leftSide)*(leftSide:t()*rightSide):reshape(4)
	local H = torch.Tensor({transf[1], -transf[2], 0, transf[3], 
							transf[2], transf[1], 0, transf[4], 
							0, 0, 1, transf[4],
							0,0,0,0,1}):reshape(4,4)
	return H
end

--returns -1 if function diverges
--parameters should have: max_iterations, max_distance, max_angle, max_translation
function ICP2D.align(points1, points2, parameters)
	local max_iterations = parameters.max_iterations or 50
	local max_distance = parameters.max_distance or 2
	local max_angle = parameters.max_angle or 5
	local max_translation = parameters.max_translation or .1 --.1 = 10 cm
	local iterations = 0
	local score = 0
	local H
	if(points1:size(2) == 2) then
		H = torch.eye(3)
	else
		H = torch.eye(4)
	end
	while(iterations<max_iterations) do
		print(iterations)
		local ind, dis = opencv.flann.flann_knn(opencv.Mat.new(points1), opencv.Mat.new(points2), 1)
		ind = ind:toTensor():long()
		dis = dis:toTensor()
		local score_n = torch.lt(dis,max_distance):sum()
		print(torch.lt(dis,max_distance):sum())
		--local score_n = dis[torch.lt(dis,max_distance)]:sum()/torch.lt(dis,max_distance):sum()
		print(score_n)
		print(score)
		if (score_n >= score) then
			score = score_n
		else 
			break
		end
		local t = image.thresholdReturnCoordinates(dis,max_distance, true):long()
		print(dis[t][torch.lt(dis[t],max_distance)]:sum())
		local points1_t = points1:index(1,t)
		local points2_t = points2:index(1,ind:index(1,t):reshape(t:size(1)))
		if(points1:size(2) == 2) then
			H=findBestTransformation(points1_t, points2_t)
			if(math.deg(torch.abs(torch.acos(H[1][1]))) > max_angle) or H[1][3] > max_translation or H[2][3] > max_translation then
				print("icp did not converge to anything useful")
				return -1
			end
		else
			H=findBestTransformation3D(points1_t, points2_t)
		end
		points1 = torch.mm(H, torch.cat(points1:t(), torch.ones(1,points1:t():size(2)),1)):t()[{{},{1,points1:size(2)}}]
		iterations = iterations+1
		print("new one")
	end
	return H
end
