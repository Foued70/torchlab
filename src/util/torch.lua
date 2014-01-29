Class()
-- Method to return pointer to raw data
function flipTB(tensor)
  local t = tensor:clone()
  for i=1,t:size(1) do
    t[i] = tensor[t:size(1)-i+1]
  end
  return t
end 

function flipLR(tensor)
  return torch.flipTB(tensor:t()):t()
end

-- TODO: description and proper input handling!
function meshgrid( xmin, xmax, ymin, ymax) 
    local xrng = xmax-xmin+1
    local yrng = ymax-ymin+1
    local xgrid = torch.range(xmin,xmax):repeatTensor(yrng,1)
    local ygrid = torch.range(ymin,ymax):repeatTensor(xrng,1)
    return xgrid, ygrid:t()
end

-- TODO: description and proper input handling!
function meshlist( xmin, xmax, ymin, ymax )
    local xgrid, ygrid = meshgrid( xmin, xmax, ymin, ymax)
    local xrng = xmax-xmin+1
    local yrng = ymax-ymin+1
    return xgrid:reshape(1,xrng*yrng), ygrid:reshape(1,xrng*yrng)
end

--[[ 
    maskdim:  -- possibly not the best naming choice
        Apply a mask along the specified dimension 
        note: Similar in functionality as select3d, probably could be made a part of that function
]]--
function maskdim( tensor, mask, dim )
    if mask:type() ~= "torch.ByteTensor" then
        error("mask is not of type torch.ByteTensor")
    end
    -- TODO: more size, spec testing, etc ...
    return tensor:index(dim,torch.range(1,tensor:size(dim)):long()[mask])
end

function unique(tensor)
	if not(tensor:type() == "torch.IntTensor") then
		error("requires int tensor")
	end
	if not(tensor:dim()==1) then
		error("tensor should be 1 dimensional")
	end
	local minV = tensor:min()
	local maxV = tensor:max()
	
	tensor = tensor - minV + 1
	local combined = _G.torch.zeros(maxV-minV+1):byte()
	combined:indexCopy(1,tensor:long(), _G.torch.ones(tensor:size()):byte())
	local t = _G.torch.range(minV,maxV)
    return t[combined]
end

function select3d(from, selectPts)
    local npts
    if(from:dim()==2) then
        if (selectPts:dim() == 1) then
            npts = selectPts:size(1)
        else
            npts = selectPts:size(1)*selectPts:size(2)
        end
        return from:index(1,_G.torch.range(1,npts)[selectPts:reshape(npts)]:long())
    elseif(from:dim()==3) then
        npts = selectPts:size(1)*selectPts:size(2)
        from = from:reshape(3,npts):t()
        return from:index(1,_G.torch.range(1,npts)[selectPts:reshape(npts)]:long())

    else
        error("wrong number of select pts")
    end

end

function assign3d(from, selectPts, normals_n)
    normals_n[{{},1}][selectPts] = from[{{},1}]
    normals_n[{{},2}][selectPts] = from[{{},2}]
    normals_n[{{},3}][selectPts] = from[{{},3}]
    return normals_n
end
