local function drawLine(img, smb, value)
	startY = 1
	endY =img:size()[2]

	startX = 1
	endX = img:size()[1]
	
	sm = smb[1]
	sb = smb[2]

	if sm~= inf then
		for i=startX, endX do
			y = sb + sm*i
			if not(y>endY) and not(y<startY) then
				img[i][y] = value
			end
		end
	end
	if sm~=0 then
		for i=startY, endY do
			x = (i-sb)/sm

			if not(x>endX) and not(x<startX) then

				img[x][i] = value
			end
		end
	end
end

local function findIntersect(smb, dmb)
	sm = smb[1]
	sb = smb[2]
	dm = dmb[1]
	db = dmb[2]
	ix = (db-sb)/(sm-dm)
	iy = sm*(ix)+sb

return ix, iy

end

local function isSameLine(smb, dmb)
	sm = smb[1]
	sb = smb[2]
	dm = dmb[1]
	db = dmb[2]

	return torch.abs(sm-dm)< .01 and torch.abs(sb-db) < 5

end

function binarySearchForClosestNumberLines(dst, minH, maxH, num_lines)
   houghThreshold = torch.floor((maxH+minH)/2)
   linesP = opencv.imgproc.HoughLinesProbabilistic(dst, 1, math.pi/360, houghThreshold, 25, 80);
   numLines = linesP:size()[2]

   if(houghThreshold == minH) or (houghThreshold == maxH) or (numLines == num_lines) then
      return linesP
   end
   if(numLines > num_lines) then
         return binarySearchForClosestNumberLines(dst, houghThreshold, maxH, num_lines)
   else
         return binarySearchForClosestNumberLines(dst, minH, houghThreshold, num_lines)
   end
end

-- use graphics magick to load the image
img_temp = image.load("/Users/stavbraun/cloudlab/tmp/arcs/motor-unicorn-0776/work/stav_test/flattened/sweep_010.png", "byte",nil,"LAB","DHW");
img_temp2 = opencv.Mat.new("/Users/stavbraun/cloudlab/tmp/arcs/motor-unicorn-0776/work/stav_test/flattened/sweep_010.png"):toTensor()
img_src = opencv.Mat.new(img_temp2:clone())
img_copy = img_temp:clone()

matMat_src = img_src:clone()
matMat_src:convert("RGB2GRAY");

dst = opencv.imgproc.CannyDetectEdges(matMat_src, 100, 200)

--slope, intersect
linesSoFar = {}

linesP = binarySearchForClosestNumberLines(matMat_src, 20, 150, 3)
	--linesP = opencv.imgproc.HoughLinesProbabilistic(dst, 1, math.pi/360, 110, 25, 5 );
linesT = linesP:toTensor()+1;
linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])
slopes = torch.cdiv((linesT[{{},1}]-linesT[{{},3}]):double(),(linesT[{{},2}]-linesT[{{},4}]):double())
intersects = linesT[{{},1}]:double()-slopes:clone():cmul(linesT[{{},2}]:double())
a = (linesT[{{},2}])

if(torch.eq(intersects,math.huge):sum()>0) then
intersects[torch.eq(intersects,math.huge)] = a[torch.eq(intersects,math.huge)] 
intersects[torch.eq(intersects,-math.huge)] = a[torch.eq(intersects,-math.huge)] 
end
for k=1,slopes:size()[1] do
	linesSoFar[table.getn(linesSoFar)+1] = torch.Tensor({slopes[k], intersects[k]})
end


counter = 2
prev_size = table.getn(linesSoFar)
while(table.getn(linesSoFar) < 40 and counter < 25) do
	linesP = binarySearchForClosestNumberLines(matMat_src, 1, 150, 3*counter)
	--linesP = opencv.imgproc.HoughLinesProbabilistic(dst, 1, math.pi/360, 110, 25, 5 );
	linesT = linesP:toTensor()+1;
	linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])
	slopes = torch.cdiv((linesT[{{},1}]-linesT[{{},3}]):double(),(linesT[{{},2}]-linesT[{{},4}]):double())
	intersects = linesT[{{},1}]:double()-slopes:clone():cmul(linesT[{{},2}]:double())
	if(torch.eq(intersects,math.huge):sum()>0) then
		intersects[torch.eq(intersects,math.huge)] = a[torch.eq(intersects,math.huge)] 
		intersects[torch.eq(intersects,-math.huge)] = a[torch.eq(intersects,-math.huge)] 
	end
	number_added=0
		for k=1,linesT:size()[1] do
			shouldAdd = true;
			for j = 1, table.getn(linesSoFar) do
				if isSameLine(linesSoFar[j], torch.Tensor({slopes[k], intersects[k]})) then
					shouldAdd = false
					break
				end
				x, y = findIntersect(linesSoFar[j], torch.Tensor({slopes[k], intersects[k]}))
				if (x>0) and (x<=img_copy:size()[2]) and (y>0) and (y<=img_copy:size()[3]) then
					angle1 = torch.atan(linesSoFar[j][1])
					angle2= torch.atan(slopes[k])
					if(angle1 < 0) then angle1=math.pi+angle1 end;
					if(angle2 < 0) then angle2=math.pi+angle2 end;
					if (torch.abs(angle1-angle2) < 2*math.pi/360*10) then
						shouldAdd = false;
					end
				end
			end
			if(shouldAdd) then
				linesSoFar[table.getn(linesSoFar)+1] = torch.Tensor({slopes[k], intersects[k]})
			end
		end

		if(table.getn(linesSoFar) == prev_size) then
			break
		else
			prev_size = table.getn(linesSoFar)
		end
	counter = counter+1
end
slopeIntersectTensor = torch.zeros(table.getn(linesSoFar), 2)
for i =1, table.getn(linesSoFar) do
	slopeIntersectTensor[{i,{}}] = linesSoFar[i]
end
print(slopeIntersectTensor)
color = torch.Tensor({0, 255, 0})

angles = torch.atan(slopeIntersectTensor[{{},1}])
angles:apply(function(val) if val<0 then return math.pi + val end end)
vals, order = torch.sort(angles)
absoluted = torch.abs(vals[{{1,vals:size()[1]-1},1}]-vals[{{2,vals:size()[1]},1}])
maxV, locV = torch.max(absoluted,1)
--vals[locV] and vals[locV+1] is the boundary


firstGroupIndices = order[{{1,locV[1]},1}]
secondGroupIndices = order[{{locV[1]+1, vals:size()[1]},1}]

for i = 1,slopeIntersectTensor:size()[1] do
	for k=1,3 do
		--print(slopeIntersectTensor[{i,{}}])
		drawLine(img_copy[k], slopeIntersectTensor[{i,{}}], color[k])
	end
end

cornerCounter = 0
for i=1, firstGroupIndices:size()[1] do
	for j=1, secondGroupIndices:size()[1] do		
		x, y = findIntersect(slopeIntersectTensor[{firstGroupIndices[i],{}}],
				slopeIntersectTensor[{secondGroupIndices[j],{}}])
		if (x>0) and (x<=img_copy:size()[2]) and (y>0 ) and (y<=img_copy:size()[3]) then
			image.addPointTo3dImage(img_copy,  opencv.imgproc.colors.MAGENTA, x,y, 2)
			cornerCounter = cornerCounter+1
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


return a;

