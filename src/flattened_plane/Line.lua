Line = Class()
local path = require 'path'
local io = require 'io'
local FlattenedPlane = flattened_plane.FlattenedPlane
function Line:__init(fname, startX,startY, endX, endY, intersectD, size_x, size_y)
    if (size_x) then
        self.size_x = size_x
        self.size_y = size_y
        if ((intersectD:dim()~=1) or (intersectD:size(1) ~= (math.max(math.abs(endX-startX), math.abs(endY-startY))+1))) then
            error("bad values")
        end
        self.values = intersectD
    else
        self.size_x = intersectD[1]:size(1)
        self.size_y = intersectD[1]:size(2)
        self.values = intersectD[2][intersectD[1]]
    end    
    self.startX = startX
    self.startY = startY
    self.endX = endX
    self.endY = endY
    if(fname) then
        self.fsave_me = fname
        self:saveMe()
    end
    --self.plane[1]:addAllFrustrumsAndEmpties()
    --    self.plane[2]:addAllFrustrumsAndEmpties()
end
function Line:__write_keys()
  return {'base_dir', 'startX', 'startY', 'endX', 'endY', 'size_x', 'size_y', 'values'}
end

function Line:saveMe()
    torch.save(self.fsave_me, self)
end
--j = 0 or 1

function Line:getLineImage()
    local x_linspace
    local xreverse = 0
    local numPixels = math.max(math.abs(self.endX-self.startX)+1, math.abs(self.endY-self.startY)+1)
    if(self.startX <= self.endX) then
        x_linspace = torch.linspace(self.startX, self.endX,numPixels)
    else
        x_linspace = torch.linspace(self.endX, self.startX,numPixels)
        xreverse = 1
    end

    local y_linspace
    local yreverse = 0
    if(self.startY <= self.endY) then
        y_linspace = torch.linspace(self.startY, self.endY,numPixels)
    else
        y_linspace = torch.linspace(self.endY, self.startY,numPixels)
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
    local t, tv =FlattenedPlane.flattened2Image(combined, torch.Tensor({1,1}):reshape(1,2), torch.Tensor({self.size_x, self.size_y}):reshape(1,2), self.values)
    return t:byte(), tv
end


