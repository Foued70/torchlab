global objects_read = Dict{Int,Any}()
force = false

function read_torch_ascii(filename)
	file_array = open(x->readdlm(x,'\n'),filename)
	return read_object(file_array, 0)
end

baremodule TORCH_TYPES
	const TYPE_NIL =0
	const TYPE_NUMBER =1
	const TYPE_STRING =2
	const TYPE_TABLE =3
	const TYPE_TORCH =4
	const TYPE_BOOLEAN =5
	const TYPE_FUNCTION =6
	
end

function read_object(A, lineidx)
	typeidx = int(A[lineidx+=1])	
	if typeidx == TORCH_TYPES.TYPE_NIL
	    return "" #as close to null as possible?
	end

	if typeidx == TORCH_TYPES.TYPE_NUMBER
	    return  float(A[lineidx+=1]), lineidx
	elseif typeidx == TORCH_TYPES.TYPE_BOOLEAN
	    return  bool(A[lineidx+=1]), lineidx
	elseif typeidx == TORCH_TYPES.TYPE_STRING
	    size =  int(A[lineidx+=1]) #the first number says how many characters are in the string
	    return  A[lineidx+=1], lineidx
	elseif typeidx == TORCH_TYPES.TYPE_FUNCTION
	    #not sure how to deal with this
	elseif typeidx == TORCH_TYPES.TYPE_TABLE || typeidx == TORCH_TYPES.TYPE_TORCH
		#raed the index
	    local index =  int(A[lineidx+=1])
	    
	    #check if it's already read
	    if haskey(objects_read,index) && !force
	    	return objects[index]
	    end

	    #otherwise read it
	    if typeidx == TORCH_TYPES.TYPE_TORCH
	    	num_char = A[lineidx+=1]
	    	version = A[lineidx+=1]
	    	versionNumber = int(match(r"^V (.*)$",version).captures[1]) #won't handle floating point numbers, it will also fail if the string doesn't match

	    	num_char = A[lineidx+=1]
	    	classname = A[lineidx+=1]
			if classname[end-5:end]=="Tensor"
	    		typename = classname[7:end-6] #"torch.XXXXXXTensor"

	    		ndims = int(A[lineidx+=1])
	    		sizetext = (ndims == 1) ? string(int(A[lineidx+=1])) : A[lineidx+=1]
	    		dims = int(matchall(r"\d+",sizetext))
	    		stridetext = A[lineidx+=1]
	    		storageOffset = A[lineidx+=1]
	    		storageObject, lineidx = read_object(A, lineidx)
	    		return reshape(copy(storageObject),dims...).', lineidx
	    	elseif classname[end-6:end]=="Storage"
	    		typename = classname[7:end-7] #torch.XXXXXXXStorage
	    		nElem = int(A[lineidx+=1])
	    		tdata = A[lineidx+=1]
	    		if typename == "Byte"
	    			temp= Array(Uint8,nElem)
	    			for i=1:nElem
	    				temp[i] = uint8(tdata[i])
	    			end
	    			return temp, lineidx
	    		elseif typename == "Int"
	    			return int32(split(tdata,' '))[1:nElem], lineidx
	    		elseif typename == "Double"
	    			return float64(split(tdata,' '))[1:nElem], lineidx
	    		elseif typename == "Long"
	    			return int64(split(tdata,' '))[1:nElem], lineidx
	    		elseif typename == "Char"
	    			return int8(split(tdata,' '))[1:nElem], lineidx
	    		elseif typename == "Short"
	    			return int16(split(tdata,' '))[1:nElem], lineidx
	    		elseif typename == "Float"
	    			return float32(split(tdata,' '))[1:nElem], lineidx
	    		else
	    			error()
	    		end

	    	else
	    		println("What is this torch thing?")
	    	end

	    elseif typeidx == TORCH_TYPES.TYPE_TABLE
	         local size = int(A[lineidx+=1])
	         local object = Dict{Any, Any}()
	         objects_read[index] = object
	         for i = 1:size
	            k, lineidx = read_object(A,lineidx)
	            v, lineidx = read_object(A,lineidx)
	            object[k] = v
	         end
	         return object, lineidx

	    end
	    #NOT FINISHED
	end
end





