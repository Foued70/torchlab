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
    yrng = ymax-ymin
    xgrid = torch.range(xmin,xmax):repeatTensor(yrng,1)
    ygrid = xgrid:clone():t()
    return xgrid, ygrid
end

-- TODO: description and proper input handling!
function meshlist( xmin, xmax, ymin, ymax )
    xgrid, ygrid = meshgrid( xmin, xmax, ymin, ymax)
    xrng = xmax-xmin
    yrng = ymax-ymin
    return xgrid:reshape(1,xrng*yrng), ygrid:reshape(1,xrng*yrng)
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
