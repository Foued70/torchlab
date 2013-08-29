local function drawLine(img, x0, y0, x1, y1, value)
	startY = 1
	endY =img:size()[2]

	startX = 1
	endX = img:size()[1]
	
	if x1~=x0 then
		for i=startX, endX do
			y = y0 + (y1-y0)/(x1-x0)*(i-x0)
			if not(y>endY) and not(y<startY) then
				img[i][y] = value
			end
		end
	end
	if y1~=y0 then
		for i=startY, endY do
			x = x0 + (x1-x0)/(y1-y0)*(i-y0)
			if not(x>endX) and not(x<startX) then
				img[x][i] = value
			end
		end
	end
end

local function findIntersect(firstCoordinates, secondCoordinates)

--first line
sx0 = firstCoordinates[2]
sy0 = firstCoordinates[1]
sx1 = firstCoordinates[4]
sy1 = firstCoordinates[3]

sm=(sy1-sy0)/(sx1-sx0)

sb = sy0-sm*sx0
dx0 = secondCoordinates[2]
dy0 = secondCoordinates[1]
dx1 = secondCoordinates[4]
dy1 = secondCoordinates[3]

dm=(dy1-dy0)/(dx1-dx0)
db = dy0-dm*dx0

ix = (db-sb)/(sm-dm)
iy = sm*(ix)+sb

return ix, iy

end
-- use graphics magick to load the image
img_temp = image.load("../../image/test/sweep_001.png", "byte",nil,"LAB","DHW");
img_temp2 = opencv.Mat.new("../../image/test/sweep_002.png"):toTensor()
img_src = opencv.Mat.new(img_temp2:clone())
matMat_src = img_src:clone()
matMat_src:convert("RGB2GRAY");

dst = opencv.imgproc.CannyDetectEdges(matMat_src, 100, 200)
linesP = opencv.imgproc.HoughLinesProbabilistic(dst, 1, math.pi/360, 80, 25, 80 );
print(linesP:size())
linesT = linesP:toTensor()+1;
linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])
img_copy = img_temp:clone()

 color = torch.Tensor({0, 255, 0})

slopes = torch.atan(torch.cdiv((linesT[{{},1}]-linesT[{{},3}]):double(),(linesT[{{},2}]-linesT[{{},4}]):double()))
slopes:apply(function(val) if val<0 then return math.pi + val end end)
vals, order = torch.sort(slopes)
absoluted = torch.abs(vals[{{1,vals:size()[1]-1},1}]-vals[{{2,vals:size()[1]},1}])
maxV, locV = torch.max(absoluted,1)
--vals[locV] and vals[locV+1] is the boundary


firstGroupIndices = order[{{1,locV[1]},1}]
secondGroupIndices = order[{{locV[1]+1, vals:size()[1]},1}]

cornerCounter = 0;
for i=1, firstGroupIndices:size()[1] do
	for j=1, secondGroupIndices:size()[1] do
		x, y = findIntersect(linesT[{firstGroupIndices[i],{}}],linesT[{secondGroupIndices[j],{}}])
		if (x>0) and (x<=img_copy:size()[2]) and (y>0 ) and (y<=img_copy:size()[3]) then
			image.addPointTo3dImage(img_copy,  opencv.imgproc.colors.MAGENTA, x,y, 2)
			cornerCounter = cornerCounter+1
		end
		for k=1,3 do
			drawLine(img_copy[k], linesT[firstGroupIndices[i]][2], linesT[firstGroupIndices[i]][1], linesT[firstGroupIndices[i]][4], linesT[firstGroupIndices[i]][3], color[k])
			drawLine(img_copy[k], linesT[secondGroupIndices[j]][2], linesT[secondGroupIndices[j]][1], linesT[secondGroupIndices[j]][4], linesT[secondGroupIndices[j]][3], color[k])
		end
		x = nil
		y = nil
	end
end
print(cornerCounter)
a = {}
a[1] = img_copy
a[2] = img_temp
image.display(a)