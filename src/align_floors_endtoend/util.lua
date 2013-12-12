Class()

function applyToPointsReturn2d(H, mat_src)
   if (mat_src:size(1) == 2) then
      return torch.mm(H, torch.cat(mat_src, torch.ones(1,mat_src:size(2)),1))[{{1,2},{}}]
   elseif (mat_src:size(1) == 3) then
      return torch.mm(H, mat_src)[{{1,2},{}}]
   end   
end
function get2DTransformationToZero(img)
   return getRotationMatrix(0,torch.Tensor({-img:size(1)/2, -img:size(2)/2}))
end

function getRotationMatrix(angle, translation)
    local cosA = math.cos(angle)
    local sinA = math.sin(angle)
    return torch.Tensor({{cosA, -sinA, translation[1]},{sinA, cosA, translation[2]},{0, 0, 1}})
end

function get2DTo3DTransformation(scale, transformation, zshift)
    local scaleT  = torch.eye(3)
    scaleT[1][1] = 1/scale
    scaleT[2][2] = 1/scale
    

    local iscaleT  = torch.eye(3)
    iscaleT[1][1] = scale
    iscaleT[2][2] = scale
    
    local rot = iscaleT*transformation * scaleT

    local combined = torch.eye(4)
    combined[{{1,2},{1,2}}] = rot[{{1,2},{1,2}}]
    combined[1][4] = rot[1][3]
    combined[2][4]= rot[2][3]
    combined[3][4] = zshift
    return combined
end

function get3Dto2DTransformation(scale, transformation)
    local scaleT  = torch.eye(3)
    scaleT[1][1] = 1/scale
    scaleT[2][2] = 1/scale
    
    local iscaleT  = torch.eye(3)
    iscaleT[1][1] = scale
    iscaleT[2][2] = scale
    
    local newTransform = torch.eye(3)
    newTransform[{{1,2},{1,2}}] = transformation[{{1,2},{1,2}}]
    newTransform[{{1,2},3}] = transformation[{{1,2},4}]
    return scaleT * newTransform * iscaleT  
end

function flattened2Image(flattenedxy,corners, minT, maxT)
    local combinedMin = torch.min(flattenedxy,1)
    local minT = minT or torch.min(combinedMin, 1)
    if corners then
        local cMin = torch.min(corners,1)
        minT[1][1] = math.min(minT[1][1],cMin[1][1])
        minT[1][2] = math.min(minT[1][2],cMin[1][2])
    end
    
    local combinedMax = torch.max(flattenedxy,1)
    local maxT = maxT or torch.max(combinedMax, 1)
    if corners then
        local cMax = torch.max(corners,1)
        maxT[1][1] = math.max(maxT[1][1],cMax[1][1])
        maxT[1][2] = math.max(maxT[1][2],cMax[1][2])
    end
    flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1
    
    local size_us = (maxT-minT+1):ceil():reshape(2)
    local combined = torch.zeros(size_us[1], size_us[2])
    for i = 1, flattenedxy:size(1) do
        combined[math.max(1,flattenedxy[i][1])][math.max(1,flattenedxy[i][2])]=1
    end

    if corners then
        corners_orig = corners
        corners = corners - torch.repeatTensor(minT, corners:size(1),1)+1
        combined = combined:repeatTensor(3,1,1)
        for i = 1, corners:size(1) do
            combined:sub(1,1,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(1)
            combined:sub(2,2,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(0)
            combined:sub(3,3,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(0)
        end
    end
    return combined
end

--warps 2 into 1 using H
function warpAndCombine(H, flattenedxy1, flattenedxy2)
    flattenedxy2 = applyToPointsReturn2d(H,flattenedxy2:t()):t()
    local src_center =    applyToPointsReturn2d(H,torch.zeros(2,1))
    local dest_center = torch.zeros(2,1)
    local combinedMin = torch.cat(torch.min(flattenedxy1,1), torch.min(flattenedxy2,1), 1)
    local minT = torch.min(combinedMin, 1):reshape(2)

    local combinedMax = torch.cat(torch.max(flattenedxy1,1), torch.max(flattenedxy2,1), 1)
    local maxT = torch.max(combinedMax, 1):reshape(2)

    local H_translation = getRotationMatrix(0,torch.Tensor({-minT[1]+1, -minT[2]+1}))

    flattenedxy1 = applyToPointsReturn2d(H_translation, flattenedxy1:t()):t()
    flattenedxy2 = applyToPointsReturn2d(H_translation,flattenedxy2:t()):t()

    local size_us = (maxT-minT+1):ceil():reshape(2)
    local su1 = size_us[1]
    local su2 = size_us[2]
    local combined = torch.zeros(3, su1, su2)

    for i = 1, flattenedxy2:size(1) do
        local f = flattenedxy2[i]
        local hh = f[1]
        local ww = f[2]
        local hh_l = math.max(1,math.floor(hh))
        local ww_l = math.max(1,math.floor(ww))
        local hh_h = math.min(su1,math.ceil(hh))
        local ww_h = math.min(su2,math.ceil(ww))

        if hh_h - hh < hh - hh_l then
            hh = hh_l
        else
            hh = hh_h
        end

        if ww_h - ww < ww - hh_l then
            ww = ww_l
        else
            ww = ww_h
        end

        combined[2][hh][ww]=255
    end
    for i = 1, flattenedxy1:size(1) do
        local f = flattenedxy1[i]
        local hh = f[1]
        local ww = f[2]
        local hh_l = math.max(1,math.floor(hh))
        local ww_l = math.max(1,math.floor(ww))
        local hh_h = math.min(su1,math.ceil(hh))
        local ww_h = math.min(su2,math.ceil(ww))

        if hh_h - hh < hh - hh_l then
            hh = hh_l
        else
            hh = hh_h
        end

        if ww_h - ww < ww - hh_l then
            ww = ww_l
        else
            ww = ww_h
        end

        combined[1][hh][ww]=255
    end
    src_center = applyToPointsReturn2d(H_translation,src_center):reshape(2)
    dest_center = applyToPointsReturn2d(H_translation,dest_center):reshape(2)
    return src_center, dest_center, combined
end

function findAngleDifference(img_src, img_dest)
   local angle_options_real = findMainDirections(img_src, img_dest)
   return math.min(angle_options_real[1], math.pi/2-angle_options_real[1])
end

function findDirections(normalList)
  local nmp = normalList:sub(1,2):clone():cmul(normalList[3]:clone():abs():lt(0.25):type('torch.DoubleTensor'):repeatTensor(2,1,1))
  nmp:cmul(normalList:clone():pow(2):sum(1):squeeze():gt(0.25):type('torch.DoubleTensor'):repeatTensor(2,1,1))
  local nmp_norm = nmp:clone():pow(2):sum(1):sqrt():squeeze()
  nmp:cdiv(nmp_norm:repeatTensor(2,1,1))
  
  local acos = nmp[1]:clone():acos()
  local asin = nmp[2]:clone():asin()
  local thetas = acos:clone():cmul(asin:ge(0):type('torch.DoubleTensor')):add(
                 acos:clone():mul(-1):add(2*math.pi):cmul(asin:lt(0):type('torch.DoubleTensor')))
  
  dirMap = torch.histc(thetas,360,0,2*math.pi)
  
  return dirMap
  
end
function findAngDiff(nlist1,nlist2,trans1,trans2)
  local t1 = trans1:clone()
  t1:sub(1,2,3,3):fill(0)
  local t2 = trans2:clone()
  t2:sub(1,2,3,3):fill(0)
  
  local nl1 = t1:clone()*nlist1:transpose(1,2)
  local nl2 = t2:clone()*nlist2:transpose(1,2)
  
  local d,d1 = findDirections(nl1):max(1)
  local d,d2 = findDirections(nl2):max(1)
  diff = d2[1]-d1[1]
  if(diff < 0) then
    diff = diff + 180
  end
  if(diff > 180) then
    diff = diff -180
  end
  return math.rad(diff)
  --diff = math.abs(d2[1]-d1[1])
  --diff = math.min(diff, math.abs(90-diff), math.abs(180-diff), math.abs(270-diff), math.abs(360-diff))*math.pi/180

--  return diff
   
end