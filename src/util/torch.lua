
torch = Class()

-- Method to return pointer to raw data
function torch.flipTB(tensor)
  local t = tensor:clone()
  for i=1,t:size(1) do
    t[i] = tensor[t:size(1)-i+1]
  end
  return t
end 

function torch.flipLR(tensor)
  return torch.flipTB(tensor:t()):t()
end


function torch.unique(tensor)
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

function torch.select3d(from, selectPts)
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