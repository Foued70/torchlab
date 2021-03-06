local hough = {}

-- load C lib

local libhough = require 'libhough'

local pi = math.pi
local twopi = 2*pi


function hough.get_hough_transform(img, numRadius, numAngles)
  local hT = torch.zeros(numRadius, numAngles);
  torch.Tensor.libhough.houghTransform(hT,img);
  hT = hT:type('torch.DoubleTensor');
  hT = hT/(hT:max()+0.00001);
  return hT;
end

function hough.get_hough_transform_vertical(img, numRadius, numAngles)
  local hT = torch.zeros(numRadius, numAngles);
  torch.Tensor.libhough.houghTransformVertical(hT,img);
  hT = hT:type('torch.DoubleTensor');
  hT = hT/(hT:max()+0.00001);
  return hT;
end

function hough.local_contrast_normalization(hT)
  local hTclone=hT:clone()
  torch.Tensor.libhough.localContrastNormalization(hT,hTclone);
  return hT;
end

function hough.restrict_angles(hT, min_ang, max_ang)
  local numRadius = hT:size(1);
  local numAngles = hT:size(2)
  local incA = (2 * pi)/numAngles
  
  local minA = math.max(1,math.floor(min_ang/incA))
  local maxA = math.min(math.ceil(max_ang/incA),numAngles)
  
  local ret = hT:clone()
  ret=ret:transpose(2,1)
  if minA > 1 then
    ret:sub(1,minA-1):fill(0.0)
  end
  if maxA < numAngles then
    ret:sub(maxA+1,numAngles):fill(0.0)
  end
  ret=ret:transpose(2,1)
  return ret
end

local function isBestInWindow(hT, r, a, winR, winA)
  local numRadius = hT:size(1);
  local numAngles = hT:size(2);
  local val = hT[r][a]
  for i = math.max(1,r-winR), math.min(r+winR, numRadius) do
    for j = -winA,winA do
      local jj = (a+j-1) % numAngles + 1
      if val < hT[i][jj] then
        return false
      end
      if (r < winR) then
      	jj = (a+j-1+(numAngles/2)) % numAngles + 1
      	if val < hT[i][jj] then
        	return false
      	end
      end
    end
  end
  
  return true
end

function hough.find_best_lines(hT, numBest)
  
  local numRadius = hT:size(1);
  local numAngles = hT:size(2);
  local ret = torch.zeros(numBest,3);
  

  for ir = 1,numRadius do
    for ja = 1,numAngles do
      local val = hT[ir][ja]
      if val > 0 and isBestInWindow(hT, ir, ja, math.ceil(numRadius/50), math.ceil(numAngles/45)) then
        for bb = 1,numBest do
          if val > ret[bb][3] then
            if bb < numBest then
              local ret1 = ret:clone()
              ret:sub(bb+1,numBest):add(0.000001):cdiv(ret:sub(bb+1,numBest)):cmul(ret1:sub(bb,numBest-1))
            end
            ret[bb]=torch.Tensor({ir,ja,val})
            break;
          end
        end
      end
    end
  end

  return ret
end

function hough.find_best_lines_vertical(hT, numBest)
  local numRadius = hT:size(1);
  local numAngles = 1;
  local ret = torch.zeros(numBest,3);
  for ir = 1,numRadius do
    for ja = 1,numAngles do
      local val = hT[ir][ja]
      if val > 0 and isBestInWindow(hT, ir, ja, math.ceil(numRadius/50), 0) then
        for bb = 1,numBest do
          if val > ret[bb][3] then
            if bb < numBest then
              local ret1 = ret:clone()
              ret:sub(bb+1,numBest):add(0.000001):cdiv(ret:sub(bb+1,numBest)):cmul(ret1:sub(bb,numBest-1))
            end
            ret[bb]=torch.Tensor({ir,ja,val})
            break;
          end
        end
      end
    end
  end
  return ret
end

function hough.draw_line (img,r,a, numRadius, numAngles, red, grn, blu)

  if (img:size():size() == 2 or (img:size() == 3 and img:size(1) == 1)) then
    img = img:repeatTensor(3,1,1)
  end
  
  local heigt = img:size(2)
  local width = img:size(3)
  local maxRadius = math.sqrt(math.pow(heigt,2)+math.pow(width,2))/2
  local ang = (a-1) * 2 * pi/(numAngles)
  local rad = (r-1) * maxRadius/(numRadius-1)
  local cosa = math.cos(ang)
  local sina = math.sin(ang)
  
  if (ang > (45.0*pi/180) and ang <= (135.0*pi/180)) or (ang > (225.0*pi/180) and ang <= (315.0*pi/180)) then
  -- horizontal ish lines
    for col=1,width do
      local x = col-width/2
      local y = (rad - (x * cosa))/sina
      local row = math.ceil(y + heigt/2)
      if row > 0 and row <= heigt then
        img[1][row][col]=1.0
        img:sub(1,1,math.max(1,row-2),math.min(heigt, row+2),math.max(1,col-1),math.min(width,col+1)):fill(red)
        img:sub(2,2,math.max(1,row-2),math.min(heigt, row+2),math.max(1,col-1),math.min(width,col+1)):fill(grn)
        img:sub(3,3,math.max(1,row-2),math.min(heigt, row+2),math.max(1,col-1),math.min(width,col+1)):fill(blu)
      end
      if row == 1 or row == heigt then
        print("[row,col] : ["..row..", "..col.."]")
      end
      if col == 1 or col == width then
        print("[row,col] : ["..row..", "..col.."]")
      end
    end
    
  else
  -- vertical ish lines
    for row=1,heigt do
      local y = row-heigt/2
      local x = (rad - (y * sina))/cosa
      local col = math.ceil(x + width/2)
      if col > 0 and col <= width then
        img[1][row][col]=1.0
        img:sub(1,1,math.max(1,row-1),math.min(heigt, row+1),math.max(1,col-2),math.min(width,col+2)):fill(red)
        img:sub(2,2,math.max(1,row-1),math.min(heigt, row+1),math.max(1,col-2),math.min(width,col+2)):fill(grn)
        img:sub(3,3,math.max(1,row-1),math.min(heigt, row+1),math.max(1,col-2),math.min(width,col+2)):fill(blu)
      end
      if row == 1 or row == heigt then
        print("[row,col] : ["..row..", "..col.."]")
      end
      if col == 1 or col == width then
        print("[row,col] : ["..row..", "..col.."]")
      end
      
    end
  
  end
  
  return img
  
end

function hough.draw_line_vertical (img,r,a, numRadius, numAngles, red, grn, blu)

  if (img:size():size() == 2 or (img:size() == 3 and img:size(1) == 1)) then
    img = img:repeatTensor(3,1,1)
  end
  
  local heigt = img:size(2)
  local width = img:size(3)
  local maxRadius = width
  local ang = 0
  local rad = (r-1) * maxRadius/(numRadius-1)
  
  for row=1,heigt do
    local y = row
    local x = rad
    local col = math.ceil(rad)
    if col - rad > 0.5 then
      col = col - 1
    end
    
    if col > 0 and col <= width then
      img[1][row][col]=1.0
      img:sub(1,1,math.max(1,row),math.min(heigt, row),math.max(1,col),math.min(width,col)):fill(red)
      img:sub(2,2,math.max(1,row),math.min(heigt, row),math.max(1,col),math.min(width,col)):fill(grn)
      img:sub(3,3,math.max(1,row),math.min(heigt, row),math.max(1,col),math.min(width,col)):fill(blu)
    else
      print("WEIRD STUFF: width - "..width..", maxRadius - "..maxRadius..", rad - "..rad..", col - "..col..", numRadius - "..numRadius)
    end
    if row == 1 or row == heigt then
      print("[row,col] : ["..row..", "..col.."]")
    end
    if col == 1 or col == width then
      print("[row,col] : ["..row..", "..col.."]")
    end

  end
  
  return img
  
end

return hough