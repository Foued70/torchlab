Hough = Class()

Hough.houghRo = 1
Hough.houghTheta = math.pi/360
Hough.houghMinLineLength = 25
Hough.houghMaxLineGap = 80
Hough.minThreshold = 10
Hough.maxThreshold = 150
Hough.defaultThreshold = 25
Hough.numLinesDesired = 40

function Hough:__init(parameters)
	for paramName in pairs(parameters) do
      if(Hough[paramName]) then
         Hough[paramName] = parameters[paramName]
      else
         error("invalid initializing of Hough with variable" .. paramName)
      end
   end
end


function Hough:getHoughCorners(img1)
	local linesXYSoFar1 = {}, linesXYSoFar2

	Hough.houghMaxLineGap = 20
	linesP = self:binarySearchForClosestNumberLines(img1, Hough.minThreshold, Hough.maxThreshold, Hough.numLinesDesired )
	slopes, intersects, linesT = Hough.getSlopeIntersectFromHoughP(linesP)
	for k=1,linesT:size()[1] do
		linesXYSoFar1[table.getn(linesXYSoFar1)+1] = linesT[{k,{}}]
	end
	local corners1 = torch.zeros(table.getn(linesXYSoFar1)*2, 2)
	
	for i =1, table.getn(linesXYSoFar1) do
		
		corners1[{2*i-1,{}}] = linesXYSoFar1[i][{{1,2}}]
		
		corners1[{2*i,{}}] = linesXYSoFar1[i][{{3,4}}]
		
	end

	return corners1
end

function Hough:getHoughLinesAndPoints(img, display)
	local linesSoFar = self:getHoughLines(img, display)
	local slopeIntersectTensor = torch.zeros(table.getn(linesSoFar), 2)

	for i =1, table.getn(linesSoFar) do
		slopeIntersectTensor[{i,{}}] = linesSoFar[i]
	end

	--find the corners! split into two perpendicular "clusters", corners are intersections of any point in one cluster with points in the other
	local angles = torch.atan(slopeIntersectTensor[{{},1}])
	angles:apply(function(val) if val<0 then return math.pi + val end end)
	local vals, order = torch.sort(angles)
	local absoluted = torch.abs(vals[{{1,vals:size()[1]-1},1}]-vals[{{2,vals:size()[1]},1}])
	local maxV, locV = torch.max(absoluted,1)
	--vals[locV] and vals[locV+1] is the boundary


	local firstGroupIndices = order[{{1,locV[1]},1}]
	local secondGroupIndices = order[{{locV[1]+1, vals:size()[1]},1}]

	if(display) then
		local img_copy = image.get3Dfrom1DCopyChannels(img:toTensor():clone())
		color = torch.Tensor({0, 255, 0})
		for i = 1,slopeIntersectTensor:size()[1] do
			for k=1,3 do
				Hough.drawLine(img_copy[k], slopeIntersectTensor[{i,{}}], color[k])
			end
		end
		image.display(img_copy)
	end

	local imgtens = img:toTensor():type('torch.DoubleTensor')
	local kern = torch.ones(3,3)
	imgtens = torch.conv2(imgtens:clone(),kern,'F'):sub(2,img:size()[1]+1,2,img:size()[2]+1)
	
	local points = torch.zeros((firstGroupIndices:size()[1])*secondGroupIndices:size()[1], 2)
	local counter = 1
	for i=1, firstGroupIndices:size()[1] do
		for j=1, secondGroupIndices:size()[1] do
			local x, y = Hough.findIntersect(slopeIntersectTensor[{firstGroupIndices[i],{}}],
			slopeIntersectTensor[{secondGroupIndices[j],{}}])
			local xc = math.ceil(x)
			local yc = math.ceil(y)
		 	local xf = math.floor(x)
		 	local yf = math.floor(y)
		 	if xc-x < x-xf then
		 		x = xc
			else
		 		x = xf
		 	end
		 	if yc-y < y-yf then
		 		y = yc
		 	else
		 		y = yf
		 	end
		 
         	if (x>0) and (x<=img:size()[1]) and (y>0 ) and (y<=img:size()[2]) then
         		if imgtens[x][y] > 0 then
	         		points[{counter,{}}] = torch.Tensor({x,y})
			        counter = counter+1
			    end
        	 end
	     end
	end
	if counter > 1 then
	 	return points[{{1,counter-1},{}}]
 	else
 		return nil
	end

end


function Hough:getHoughLines(img, display)
--slope, intersect
local linesSoFar = {}
local linesXYSoFar = {}
local linesP = self:binarySearchForClosestNumberLines(img, Hough.minThreshold, Hough.maxThreshold, 3)
local slopes, intersects, linesT = Hough.getSlopeIntersectFromHoughP(linesP)
--initialize with first round
for k=1,slopes:size()[1] do
	linesSoFar[table.getn(linesSoFar)+1] = torch.Tensor({slopes[k], intersects[k]})
	linesXYSoFar[table.getn(linesXYSoFar)+1] = linesT[{k,{}}]
end

local counter = 2
local prev_size = table.getn(linesSoFar)
while(table.getn(linesSoFar) < Hough.numLinesDesired and counter < torch.ceil(Hough.numLinesDesired/3)) do
	linesP = self:binarySearchForClosestNumberLines(img, Hough.minThreshold, Hough.maxThreshold, 3*counter)
	slopes, intersects, linesT = getSlopeIntersectFromHoughP(linesP)
	local number_added=0
	for k=1,linesP:size()[2] do
		local shouldAdd = true;
		for j = 1, table.getn(linesSoFar) do
			if Hough.isSameLine(linesSoFar[j], torch.Tensor({slopes[k], intersects[k]})) then
				shouldAdd = false
				break
			end
			local x, y = Hough.findIntersect(linesSoFar[j], torch.Tensor({slopes[k], intersects[k]}))
			if (x>0) and (x<=img:size()[1]) and (y>0) and (y<=img:size()[2]) then
				local angle1 = torch.atan(linesSoFar[j][1])
				local angle2= torch.atan(slopes[k])
				if(angle1 < 0) then angle1=math.pi+angle1 end;
				if(angle2 < 0) then angle2=math.pi+angle2 end;
				if (torch.abs(angle1-angle2) < 2*math.pi/360*10) then
					shouldAdd = false;
				end
			end
			   
		end
		if(shouldAdd) then
			linesSoFar[table.getn(linesSoFar)+1] = torch.Tensor({slopes[k], intersects[k]})
			linesXYSoFar[table.getn(linesXYSoFar)+1] = linesT[{k,{}}]
		end
	end

	if(table.getn(linesSoFar) == prev_size) then
		break
	end
	prev_size = table.getn(linesSoFar)
	counter = counter+1
end
return linesSoFar, linesXYSoFar
end

function Hough:getMainDirections(img)
	
	local imgorig = img:clone()
	
	local linesP = opencv.imgproc.HoughLinesProbabilistic(img, Hough.houghRo, Hough.houghTheta, Hough.defaultThreshold, Hough.houghMinLineLength, Hough.houghMaxLineGap);
	--local linesP = opencv.imgproc.HoughLinesRegular(img, Hough.houghRo, Hough.houghTheta, Hough.defaultThreshold, 1,1);
	
	if linesP:size(1) == 0 then
		print('getMainDirections 2')
		print(img:size())
		print(imgorig:size())
		print(linesP:size())
		--image.display(imgorig:toTensor())
	end
	
	local slopes, intersects = Hough.getSlopeIntersectFromHoughP(linesP)
	--find the corners! split into two perpendicular "clusters", corners are intersections of any point in one cluster with points in the other
	local angles = torch.atan(slopes)
	 angles:apply(function(val) if val<0 then return math.pi + val end end)
	local vals, order = torch.sort(angles)
	local absoluted = torch.abs(vals[{{1,vals:size()[1]-1},1}]-vals[{{2,vals:size()[1]},1}])
	local maxV, locV = torch.max(absoluted,1)
	--vals[locV] and vals[locV+1] is the boundary


	local firstGroup = vals[{{1,locV[1]},1}]
	local secondGroup = vals[{{locV[1]+1, vals:size()[1]},1}]

	return torch.mean(firstGroup), torch.mean(secondGroup)
end
function Hough.drawLine(img, smb, value)
	local startY = 1
	local endY =img:size()[2]

	local startX = 1
	local endX = img:size()[1]
	
	local sm = smb[1]
	local sb = smb[2]

	if sm~= inf then
		for i=startX, endX do
			local y = sb + sm*i
			if not(y>endY) and not(y<startY) then
				img[i][y] = value
			end
		end
	end
	if sm~=0 then
		for i=startY, endY do
			local x = (i-sb)/sm
			if not(x>endX) and not(x<startX) then
				img[x][i] = value
			end
		end
	end
end

function Hough.findIntersect(smb, dmb)
	local sm = smb[1]
	local sb = smb[2]
	local dm = dmb[1]
	local db = dmb[2]
	local ix = (db-sb)/(sm-dm)
	local iy = sm*(ix)+sb
	return ix, iy
end

function Hough.isSameLine(smb, dmb)
	local sm = smb[1]
	local sb = smb[2]
	local dm = dmb[1]
	local db = dmb[2]
	return torch.abs(sm-dm)< .01 and torch.abs(sb-db) < 5

end

function Hough:binarySearchForClosestNumberLines(dst, minH, maxH, num_lines)
	local houghThreshold = torch.floor((maxH+minH)/2)
	local linesP = opencv.imgproc.HoughLinesProbabilistic(dst, Hough.houghRo, Hough.houghTheta, houghThreshold, Hough.houghMinLineLength, Hough.houghMaxLineGap);

	local numLines = linesP:size()[2]

	if(houghThreshold == minH) or (houghThreshold == maxH) or (numLines == num_lines) then
		return linesP
	end
	if(numLines > num_lines) then
		return self:binarySearchForClosestNumberLines(dst, houghThreshold, maxH, num_lines)
	else
		return self:binarySearchForClosestNumberLines(dst, minH, houghThreshold, num_lines)
	end
end


---helper to take opencv format and return something more useul, two tensors, one with slopes other with intersects
function Hough.getSlopeIntersectFromHoughP(linesP)
	
	if linesP:size(3) == 4 then
		local linesT = linesP:toTensor()+1;
		linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])
		local slopes = torch.cdiv((linesT[{{},1}]-linesT[{{},3}]):double(),(linesT[{{},2}]-linesT[{{},4}]):double())
		local intersects = linesT[{{},1}]:double()-slopes:clone():cmul(linesT[{{},2}]:double())

		if(torch.eq(intersects,math.huge):sum()>0) then
			a = (linesT[{{},2}])
			intersects[torch.eq(intersects,math.huge)] = a[torch.eq(intersects,math.huge)]:double()
			intersects[torch.eq(intersects,-math.huge)] = a[torch.eq(intersects,-math.huge)]:double()
		end
		linesT_12 = torch.cat(linesT[{{},2}], linesT[{{},1}], 2)
		linesT_34 = torch.cat(linesT[{{},4}], linesT[{{},3}], 2)
		linesT = torch.cat(linesT_12, linesT_34,2)
		return slopes, intersects, linesT
	elseif linesP:size(3) == 2 then
		local linesT = linesP:toTensor()+1;
		linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])
		local slopes = torch.Tensor(linesT[{{},2}]:clone():double():size()):fill(-1):cdiv(linesT[{{},2}]:clone():double():tan())
		local intersects = linesT[{{},1}]:clone():double():cdiv(linesT[{{},2}]:clone():double():sin())
		
		return slopes, intersects, linesT
	end
end 