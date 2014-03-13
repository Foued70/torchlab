<<<<<<< HEAD
global debug_enabled = false
global objects_read = Dict{Int,Any}()
=======
global objects_read
>>>>>>> 766d53f3fdd52d53d8473f65037c966418966bae
force = false

function read_torch_ascii(filename)
	file_array = open(x->readdlm(x,'\n'),filename)
	global objects_read = Dict{Int,Any}()
	return read_object(file_array, 0)[1]
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
	debugprnt("\ntypeidx=",typeidx)
	if typeidx == TORCH_TYPES.TYPE_NIL
	    return Nothing #as close to null as possible?
	end

	if typeidx == TORCH_TYPES.TYPE_NUMBER
		debugprnt("NUMBER=",float(A[lineidx+1]))
	    return  float(A[lineidx+=1]), lineidx
	elseif typeidx == TORCH_TYPES.TYPE_BOOLEAN
		debugprnt("BOOLEAN=",bool(A[lineidx+1]))
	    return  bool(A[lineidx+=1]), lineidx
	elseif typeidx == TORCH_TYPES.TYPE_STRING
	    size =  int(A[lineidx+=1]) #the first number says how many characters are in the string
		debugprnt("STRING",size,"=",A[lineidx+1])
	    return  A[lineidx+=1], lineidx
	elseif typeidx == TORCH_TYPES.TYPE_FUNCTION
	    #not sure how to deal with this
	elseif typeidx == TORCH_TYPES.TYPE_TABLE || typeidx == TORCH_TYPES.TYPE_TORCH
		#raed the index
	    local index =  int(A[lineidx+=1])
	    debugprnt("index=",index)
	    
	    #check if it's already read
	    if haskey(objects_read,index) && !force
<<<<<<< HEAD
	    	debugprnt("already read ",index)
	    	return objects_read[index],lineidx
=======
	    	error("not yet tested")
	    	return objects_read[index]
>>>>>>> 766d53f3fdd52d53d8473f65037c966418966bae
	    end

	    #otherwise read it
	    if typeidx == TORCH_TYPES.TYPE_TORCH
	    	num_char = A[lineidx+=1]
	    	debugprnt("num_char=",num_char)
	    	version = A[lineidx+=1]
	    	debugprnt("version=",version)
	    	versionNumber = int(match(r"^V (.*)$",version).captures[1]) #won't handle floating point numbers, it will also fail if the string doesn't match
	    	debugprnt("versionNumber=",versionNumber)

	    	num_char = A[lineidx+=1]
	    	debugprnt("num_char=",num_char)
	    	classname = A[lineidx+=1]
	    	debugprnt("classname=",classname)
			if classname[end-5:end]=="Tensor"
	    		typename = classname[7:end-6] #"torch.XXXXXXTensor"

	    		ndims = int(A[lineidx+=1])
	    		debugprnt("ndims=",ndims)
	    		sizetext = A[lineidx+=1]
	    		debugprnt("sizetext=",sizetext)
	    		dims = (ndims == 1) ? int([sizetext]) : int(matchall(r"\d+",sizetext))
	    		debugprnt("dims=",dims)
	    		stridetext = A[lineidx+=1]
	    		debugprnt("stridetext=",stridetext)
	    		storageOffset = A[lineidx+=1]
	    		debugprnt("storageOffset=",storageOffset)
	    		storageObject, lineidx = read_object(A, lineidx)
<<<<<<< HEAD
	    		debugprnt("length(storageObject)=",length(storageObject))
	    		nElem = 1
	    		for i = 1:ndims
	    			nElem *= dims[i]
	    		end
	    		debugprnt("nElem=",nElem)
=======
>>>>>>> 766d53f3fdd52d53d8473f65037c966418966bae
	    		if (ndims == 1)
	    			objects_read[index] = reshape(copy(storageObject[storageOffset - 1 + (1:nElem)]),dims...)
	    		elseif (ndims ==2)
<<<<<<< HEAD
	    			objects_read[index] = reshape(copy(storageObject[storageOffset - 1 + (1:nElem)]),dims...).'
=======
	    			return reshape(copy(storageObject),dims[2], dims[1]).', lineidx

>>>>>>> 766d53f3fdd52d53d8473f65037c966418966bae
	    		elseif (ndims == 3)
	    			#return reshape(copy(storageObject),dims...), lineidx
					objects_read[index] = permutedims(reshape(copy(storageObject[storageOffset - 1 + (1:nElem)]),dims[3],dims[2],dims[1]),[2,1,3])
	    		else
	    			error("dont support arrays with more than 3 dimensions")
	    		end

	    	elseif classname[end-6:end]=="Storage"
	    		typename = classname[7:end-7] #torch.XXXXXXXStorage
	    		nElem = int(A[lineidx+=1])
	    		debugprnt("nElem=",nElem)
	    		tdata = A[lineidx+=1]
	    		debugprnt("tdata=",tdata)
	    		if typename == "Byte"
	    			temp= Array(Uint8,nElem)
	    			for i=1:nElem
	    				temp[i] = uint8(tdata[i])
	    			end
	    			objects_read[index] = temp
	    		elseif typename == "Int"
	    			objects_read[index] = int32(split(tdata,' '))[1:nElem]
	    		elseif typename == "Double"
	    			objects_read[index] = float64(split(tdata,' '))[1:nElem]
	    		elseif typename == "Long"
	    			objects_read[index] = int64(split(tdata,' '))[1:nElem]
	    		elseif typename == "Char"
	    			objects_read[index] = int8(split(tdata,' '))[1:nElem]
	    		elseif typename == "Short"
	    			objects_read[index] = int16(split(tdata,' '))[1:nElem]
	    		elseif typename == "Float"
	    			objects_read[index] = float32(split(tdata,' '))[1:nElem]
	    		else
	    			error("bad type")
	    		end
	    	else
	    		error("What is this torch thing? "*classname)
	    	end

	    elseif typeidx == TORCH_TYPES.TYPE_TABLE
	         local size = int(A[lineidx+=1])
	         local object = Dict{Any, Any}()
	         for i = 1:size
	            k, lineidx = read_object(A,lineidx)
	            v, lineidx = read_object(A,lineidx)
	            object[k] = v
	         end
	         objects_read[index] = object

	    end
	    return objects_read[index],lineidx

	end
end

function debugprnt(x...)
	if debug_enabled
		println(x)
	end
end



