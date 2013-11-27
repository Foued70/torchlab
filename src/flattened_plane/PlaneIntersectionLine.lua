PlaneIntersectionLine = Class()
local path = require 'path'
local pcl = PointCloud.PointCloud
local io = require 'io'
local FlattenedPlane = flattened_plane.FlattenedPlane
function PlaneIntersectionLine:__init(i,j, plane1, plane2)
    local plane1 = plane1 or FlattenedPlane.get_ith_plane(i)
    local plane2 = plane2 or FlattenedPlane.get_ith_plane(j)
    local eqn1 = plane1:getPlaneEquation(i)
    local eqn2 = plane1:getPlaneEquation(j)    
    self.eq = {{PlaneIntersectionLine.getRotatedEquation(eqn1, plane1.quat),PlaneIntersectionLine.getRotatedEquation(eqn1, plane2.quat)},{PlaneIntersectionLine.getRotatedEquation(eqn2, plane1.quat),PlaneIntersectionLine.getRotatedEquation(eqn2, plane2.quat)}}
    self.plane = {}
    self.plane[1] = plane1
    self.plane[2] = plane2
    --self.plane[1]:addAllFrustrumsAndEmpties()
    --    self.plane[2]:addAllFrustrumsAndEmpties()
end

--j = 0 or 1
function PlaneIntersectionLine:convertIntersectLineToPlane(j, curj, intersect, intersectv)
    --curj= (j-1)*-1+2
    local minT, maxT = self.plane[curj]:calculateMinAndMaxT()
    local temp, temp, x_mat, y_mat = FlattenedPlane.flattened2Image(minT, minT, maxT)
    local eq_new = self.eq[curj][curj]
    --local intersect, coord = self:findIntersectionImageOnPlane((j-1)*-1+2)
    --intersectv = torch.ones(intersect[intersect]:size()):fill(10) --to do???
    
    local x = x_mat[intersect]*self.plane[curj].resolution --add min??
    local y = y_mat[intersect]*self.plane[curj].resolution
    local z = (x*eq_new[1]+y*eq_new[2]+eq_new[4])/-eq_new[3]
    local unrotate_pts = geom.quaternion.rotate(geom.quaternion.inverse(self.plane[curj].quat), torch.cat(torch.cat(x,y ,2),z,2))

    local minT, maxT = self.plane[j]:calculateMinAndMaxT()
    local rotate_pts =  geom.quaternion.rotate(self.plane[j].quat, unrotate_pts)/self.plane[j].resolution
    local good_index = torch.eq(torch.ge(rotate_pts:sub(1,-1,1,1),minT[1][1])+torch.le(rotate_pts:sub(1,-1,1,1),maxT[1][1])+
        torch.ge(rotate_pts:sub(1,-1,2,2),minT[1][2])+torch.le(rotate_pts:sub(1,-1,2,2),maxT[1][2]),4):squeeze()
    if(good_index:sum()==0) then
        return nil
    end
    local good_pts = rotate_pts:index(1,torch.range(1,good_index:size(1))[good_index]:long())
    local temp, temp, plane_coords = self:findIntersectionImageOnPlane(j)
    local distances = geom.util.pairwise_distance(good_pts:sub(1,-1,1,2), plane_coords)
    local temp, dcoords = distances:min(2)
    dcoords = dcoords:squeeze()
    intersectv = intersectv[intersect]
    local counter = 1
    local predictedF = torch.ones(plane_coords:size(1)):fill(-1)
    predictedF[dcoords:long()]=intersectv
    local predictedB = torch.ones(plane_coords:size(1)):fill(-1)
    predictedB[dcoords:long()]=intersectv
    local lastGoodValueF = 1
    local lastGoodValueB = 1
    for i =1,plane_coords:size(1) do

        if(predictedF[i] == -1) then
            predictedF[i] = lastGoodValueF
        else
            lastGoodValueF = predictedF[i]
        end
        j=plane_coords:size(1)-i+1
        if(predictedB[j] == -1) then
            predictedB[j] = lastGoodValueB
        else
            lastGoodValueB = predictedB[j]
        end

    end
    local flat2, flat2d, t, x_mat, y_mat = FlattenedPlane.flattened2Image(plane_coords, minT, maxT,
        (predictedF+predictedB)/2)
    return flat2, flat2d, plane_coords[1][1]-minT[1][1]+1, plane_coords[1][2]-minT[1][2]+1, plane_coords[-1][1]-minT[1][1]+1, plane_coords[-1][2]-minT[1][2]+1
end


--image.display(t:sum(1):squeeze()+plane.iempties)
--returns intersection image same size an plane j's image and the coordinates of the corners on this image
function PlaneIntersectionLine:findIntersectionImageOnPlane(j)
    local minT, maxT = self.plane[j]:calculateMinAndMaxT()
    local t1, t2, x_mat, y_mat = FlattenedPlane.flattened2Image(minT, minT, maxT)  
    local eq1 = self.eq[j][j]
    local eq2 = self.eq[(j-1)*-1+2][j] --j=1 --> 2, j=2-->1
    local z = -eq1[4]/eq1[3]
    local minTNew = minT:squeeze()*self.plane[j].resolution
    local maxTNew = maxT:squeeze()*self.plane[j].resolution
    local myTensor = torch.DoubleTensor({-(eq2[4]+eq2[3]*z + eq2[2]*minTNew[2])/eq2[1], minTNew[2], 
                    -(eq2[4]+eq2[3]*z + eq2[2]*maxTNew[2])/eq2[1], maxTNew[2],
                    minTNew[1],-(eq2[4]+eq2[3]*z + eq2[1]*minTNew[1])/eq2[2],
                    maxTNew[1],-(eq2[4]+eq2[3]*z + eq2[1]*maxTNew[1])/eq2[2]
                    }):reshape(4,2)
    local x_good = torch.eq(torch.ge(myTensor:select(2,1),minTNew[1]):double()+torch.le(myTensor:select(2,1),maxTNew[1]):double(),2)
    x_good[3] = 1
    x_good[4]=1

    local y_good = torch.eq(torch.ge(myTensor:select(2,2),minTNew[2])+torch.le(myTensor:select(2,2),maxTNew[2]),2)
    y_good[1] = 1
    y_good[2] = 1

    local super_good = (torch.eq(x_good+y_good,2))
    if super_good:sum() == 2 then
        local good_indices = torch.range(1,4)[super_good]
        local startX = myTensor[good_indices[1]][1]
        local endX = myTensor[good_indices[2]][1]
        local startY = myTensor[good_indices[1]][2]
        local endY = myTensor[good_indices[2]][2]
        local combined = PlaneIntersectionLine.getLineCoordinates(startX/self.plane[j].resolution, startY/self.plane[j].resolution, endX/self.plane[j].resolution, endY/self.plane[j].resolution)
        local a1,a2,a3 = FlattenedPlane.flattened2Image(combined, minT, maxT)
        return a1, torch.Tensor({startX, startY, endX, endY}), combined, torch.Tensor({startX/self.plane[j].resolution-minT[1][1], startY/self.plane[j].resolution-minT[1][2], endX/self.plane[j].resolution-minT[1][1], endY/self.plane[j].resolution-minT[1][2]})+1

    else
        print("did not intersect with this plane in given range!!!")
        return nil
    end
end


local function calculateMinAndMax(lua_table_of_vectors, minT, maxT)
    minT = minT or torch.Tensor({math.huge, math.huge, math.huge}):reshape(1,3)
    maxT = maxT or torch.Tensor({-math.huge, -math.huge, -math.huge}):reshape(1,3)
    local v = lua_table_of_vectors
    if(v:dim()~=0) then
        if(v:dim() == 1) then
            v = v:reshape(1,3)
        end
        minT = torch.min(torch.cat(minT, v,1),1)
        maxT = torch.max(torch.cat(maxT, v,1),1) 
    end
    return minT, maxT
end
function PlaneIntersectionLine:getRotatedIntersectionAndOccupiedEmpties(j)
    local intersect, coord = self:findIntersectionImageOnPlane(j)
    if(not(intersect)) then
        return nil
    end
    local occ = torch.gt(self.plane[j].ioccupied+self.plane[j].ioccupiedF,0)
    local empt = torch.gt(self.plane[j].iempties+self.plane[j].iemptiesF,0)
    local minTO, maxTO = self.plane[j]:calculateMinAndMaxT()

    local corner1 = (coord:sub(1,2)/self.plane[j].resolution-minTO+1)
    local corner2 = (coord:sub(3,4)/self.plane[j].resolution-minTO+1)
    local slope = (corner1[2]-corner2[2])/(corner1[1]-corner2[1])
    local deg = math.atan(1/slope) --(5,1 1,5) --> -45 deg, --(1,5, 5,9)-> -->45,  
    local t1, t2, x_mat, y_mat = FlattenedPlane.flattened2Image(minTO, minTO, maxTO)    
    H = torch.eye(3)
    H[1][1]= math.cos(deg)
    H[2][2] = H[1][1]
    H[1][2] = -math.sin(deg)
    H[2][1] = -H[1][2]

    --rotate so vertically aligned with plane
    local rotatePtsOcc = (H*(torch.cat(torch.cat(x_mat[occ],y_mat[occ],2), torch.ones(occ:sum()),2)):t()):t()
    local rotatePtsEmpt = (H*(torch.cat(torch.cat(x_mat[empt],y_mat[empt],2), torch.ones(empt:sum()),2)):t()):t()
    
    local corners = torch.Tensor({{coord[1], coord[3]}, {coord[2], coord[4]}, {1, 1}})/self.plane[j].resolution
    local rotatedCorners = (H*corners):t()

    local minT, maxT = calculateMinAndMax(rotatePtsOcc)
    minT, maxT = calculateMinAndMax(rotatePtsEmpt, minT, maxT)
    minT, maxT = calculateMinAndMax(rotatedCorners, minT, maxT)
       

    local startX = rotatedCorners[1][1]
    local startY = rotatedCorners[1][2]
    local endX = rotatedCorners[2][1]
    local endY = rotatedCorners[2][2]
    local combined = PlaneIntersectionLine.getLineCoordinates(startX, startY, endX, endY)

    local flatOcc, t, x_mat, y_mat= FlattenedPlane.flattened2Image(rotatePtsOcc:sub(1,-1,1,2), minT:sub(1,-1,1,2), maxT:sub(1,-1,1,2))    
    local flatEmpt= FlattenedPlane.flattened2Image(rotatePtsEmpt:sub(1,-1,1,2), minT:sub(1,-1,1,2), maxT:sub(1,-1,1,2))    
    local locOfRow = combined:t()[1]:mean()-minT[1][1]+1
    local flatIntersect = torch.zeros(flatOcc:size())
    flatIntersect[{{locOfRow, locOfRow}, {math.min(startY, endY)-minT[1][2]+1, math.max(startY, endY)-minT[1][2]+1}}]=1
    
    return flatIntersect, flatOcc, flatEmpt, x_mat, y_mat
end


--[[
intersect13 = flattened_plane.PlaneIntersectionLine.new(1,3)
flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect13:getRotatedIntersectionAndOccupiedEmpties(1)
 cum, cumO, cumE, cumUnknown = intersect13:temp(1,flatOcc, flatEmpt,flatIntersect)

    flatIntersectD = torch.zeros(flatIntersect:size())
temp,val = flatIntersect:sum(2):squeeze():max(1)
    flatIntersectD[val:squeeze()] = cum:double()
    newCum, newCumD = intersect13:sendRotateScoreBackToPlane(1, flatIntersect:byte(), flatIntersectD[flatIntersect:byte()])
]]--
function PlaneIntersectionLine:sendRotateScoreBackToPlane(j, flatIntersect, flatIntersectD, x_mat, y_mat)
    local intersect, coord = self:findIntersectionImageOnPlane(j)
    local minTO, maxTO = self.plane[j]:calculateMinAndMaxT()
    local corner1 = (coord:sub(1,2)/self.plane[j].resolution-minTO+1)
    local corner2 = (coord:sub(3,4)/self.plane[j].resolution-minTO+1)
    local slope = (corner1[2]-corner2[2])/(corner1[1]-corner2[1])
    local deg = -math.atan(1/slope)
    H = torch.eye(3)
    H[1][1]= math.cos(deg)
    H[2][2] = H[1][1]
    H[1][2] = -math.sin(deg)
    H[2][1] = -H[1][2]

    local rotatePtsBack = (H*(torch.cat(torch.cat(x_mat[flatIntersect],y_mat[flatIntersect],2), torch.ones(flatIntersect:sum()),2)):t()):t():ceil()
    local good_pts = torch.ge(rotatePtsBack:t()[1], minTO[1][1])+torch.le(rotatePtsBack:t()[1], maxTO[1][1])+
                torch.ge(rotatePtsBack:t()[2], minTO[1][2])+torch.le(rotatePtsBack:t()[2], maxTO[1][2])
    good_pts = torch.eq(good_pts,4)
    local rotatePtsBack = FlattenedPlane.select3d(rotatePtsBack, good_pts)

    local newCum, newCumD = FlattenedPlane.flattened2Image(rotatePtsBack:sub(1,-1,1,2):clone(), minTO:sub(1,-1,1,2), maxTO:sub(1,-1,1,2), 
        flatIntersectD[good_pts])    

    --image.display(temp)
    return newCum, newCumD
end
function PlaneIntersectionLine.getLineCoordinates(startX, startY, endX, endY, minT, maxT)
    local x_linspace
    local xreverse = 0
    local numPixels = math.max(math.abs(endX-startX)+1, math.abs(endY-startY)+1)
    if(startX <= endX) then
        x_linspace = torch.linspace(startX, endX,numPixels)
    else
        x_linspace = torch.linspace(endX, startX,numPixels)
        xreverse = 1
    end

    local y_linspace
    local yreverse = 0
    if(startY <= endY) then
        y_linspace = torch.linspace(startY, endY,numPixels)
    else
        y_linspace = torch.linspace(endY, startY,numPixels)
        yreverse = 1
    end
    local combined 
    if(xreverse + yreverse == 1) then
        y_linspacer = torch.range(1,y_linspace:size(1))
        y_linspacer:apply(function(v) 
                y_linspacer[v] = y_linspace[y_linspace:size(1)-v+1]
            end)
        combined = torch.cat(x_linspace,y_linspacer,2)
    else
        combined = torch.cat(x_linspace,y_linspace,2)
    end
    return combined
end



function PlaneIntersectionLine.getRotatedEquation(eqn1, quat)
    local n1 = geom.quaternion.rotate(quat, eqn1:sub(1,3))    
    local p1,p2,v = FlattenedPlane.findVector(eqn1)
    p1 = geom.quaternion.rotate(quat,p1)
    return torch.cat(n1:squeeze(),torch.Tensor({-p1*n1}),1)
end




