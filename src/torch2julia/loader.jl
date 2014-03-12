function read_torch_ascii(filename)
	file_array = open(x->readdlm(x,'\n'),filename)
	return read_object(file_array)
end

objects_read = Dict{Int,Any}()
force = false

function read_object(A)
	lineidx = 0
	read_next_expr = :(A[lineidx+=1])
	read_next = function() eval(read_next_expr) end

	typeidx = read_next()
	if typeidx == TYPE_NIL
	    return Nothing
	end

	if typeidx == TYPE_NUMBER
	    return read_next()
	elseif typeidx == TYPE_BOOLEAN
	    return read_next()
	elseif typeidx == TYPE_STRING
	    size = read_next() #the first number says how many characters are in the string
	    return read_next()
	elseif typeidx == TYPE_FUNCTION
	    #not sure how to deal with this
	elseif typeidx == TYPE_TABLE || typeidx == TYPE_TORCH
		#raed the index
	    index = read_next()
	    
	    #check if it's already read
	    if haskey(objects_read,index) && !force
	    	return objects[index]
	    end

	    #otherwise read it
	    if typeidx == TYPE_TORCH
	    	num_char = read_next()
	    	version = read_next()
	    	versionNumber = int(match(r"^V (.*)$",version).captures[1]) #won't handle floating point numbers, it will also fail if the string doesn't match

	    	num_char = read_next()
	    	classname = read_next()
	    	typename = classname[7:end-6] #"torch.XXXXXXTensor"

	    	ndims = read_next()
	    	sizetext = read_next()
	    	size = int(matchall(r"\d",sizetext))

	    	stridetext = read_next()
	    	storageOffset = read_next()

	    	storageObject = read_object(A[lineidx+1:end])
	    	if typename == "Byte"
	    	object = Array{Bool,}
	    end
	    #NOT FINISHED
	end
end





