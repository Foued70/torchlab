self = {}

function self.unique_rows(A)
	--returns unique rows of A, and the indices of these unique rows.
	--it keeps the order of the rows the same and keeps the first instance of non-unique rows
	local A = A:clone()
	local inds = torch.zeros(A:size(1))
	local cur_ind = 1
	--if the size of the second dimension is 1 then find the unique items and return the values, indices
	if A:dim()==1  or A:size(2)==1 then
		_, u_inds = self.unique(A)
		return A[u_inds]:clone():contiguous(),u_inds
	else
		local uA,ui = self.unique(A[{{},1}]:clone())
		for i = 1,uA:size(1) do
			--find the unique_rows of all the rows where the first element is equal to uA[i]
			local sub_mask = A[{{},1}]:eq(uA[i])
			local sub_inds = torch.range(1,A:size(1))[sub_mask]:long() --indices of A where the first element is all the same
			local _, u_sub_inds = self.unique_rows(A[sub_inds][{{},{2,-1}}]:clone())
			
			inds[{{cur_ind,cur_ind+u_sub_inds:size(1)-1}}] = sub_inds[u_sub_inds]
			cur_ind = cur_ind+u_sub_inds:size(1)
		end
		inds  = torch.sort(inds[{{1,cur_ind-1}}]:clone(),1):long()
		return A[inds]:clone(),inds
	end
end

function self.unique(A)
	--returns the unique values of the vector A
	--A is a 1D tensor
	if type(A)~="userdata" or string.sub(A:type(),-6)~="Tensor" or (A:dim()>1 and A:size(2)>1)then
		error("Input a 1D Tensor")
	end

	if  A:size(1)>1 then
		B,i = torch.sort(A:clone(),1)
		dB = torch.add(B[{{2,-1}}],-B[{{1,-2}}])
		mask = torch.cat(torch.ByteTensor{1},dB:gt(0),1) --always include the first one, and the ones with a non-zero change to the next element
		inds = torch.range(1,B:size(1))[mask]:long()
		return B[inds],i[inds]:reshape(i[inds]:size(1))
	elseif A:size(1)==1 then
		return A:clone(), torch.LongTensor{1}
	end
	
end

function self.ANY(A,_dim)
	if _dim then
		return A:clone():gt(0):sum(_dim):gt(0)
	else
		return A:clone():gt(0):sum() > 0
	end
end

function self.ALL(A,_dim)
	if _dim then
		return A:clone():gt(0):sum(_dim):eq(A:size(_dim))
	else
		return A:clone():gt(0):sum() == A:nElement()
	end
end

function self.AND(A,B)
	return torch.add(A,B):eq(2)
end

function self.OR(A,B)
	return torch.add(A,B):ge(1)
end

function self.NOR(A,B)
	return torch.add(A,B):eq(0)
end

function self.XNOR(A,B)
	return torch.eq(A,B)
end

function self.XOR(A,B)
	return torch.ne(A,B)
end

function self.NOT(A)
	return torch.eq(A,0)
end

return self