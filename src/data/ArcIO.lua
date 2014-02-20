--[[
	ArcIO:
		Simple utility for reading and writing Arc data
		- Currently there is no magic, but eventually I imagine that
		  one could specify the current job_id and work_id they are on
		- Also it could be cool to have specs for different types of work 
		  that can be loaded and make ArcIO more automatic 
]]--


io = require('io')

--	given environment variable return value
function getenvpath( env_var ) 
	fh = io.popen("env")
	env_str = fh:read("*a")
	fh:close()
	start,_= string.find(env_str, env_var)
	if start == nil then
		print("Error env_var not found: ", env_var)
		return
	end
	_,stop = string.find(env_str, "\n", start+1)

	env_str_len = string.len(env_var)	
	return string.sub(env_str, start+env_str_len+1,stop-1)
end

-- ArcIO Class 
ArcIO = Class()

function ArcIO:__init( job_id, work_id )
	-- Get arc path 
	self.arc_path = getenvpath("ARC_PATH")	
	self.job_path = self.arc_path .. "/" .. job_id 
	self.pointcloud_path = self.job_path .. "/source/po_scan/a/"
	self.output_dir = self.job_path .. '/work/' .. work_id .. "/"
	self.scan_num = nil 

	self.job_id = job_id
	self.work_id = work_id

	print("pointcloud_path: ", self.pointcloud_path)
	print("output_dir: ", self.output_dir)
end

-- Shortcut for loading point cloud scan data 
function ArcIO:getScan( scan_num )
	self.scan_num = scan_num
	scan_fname = self.pointcloud_path .. string.format('%.3d', scan_num)
	print("Loading scan: ", scan_fname)
	loader = pointcloud.loader.load_pobot_ascii( scan_fname )
	return pointcloud.pointcloud.new(loader) 
end

-- Get string to dir in work 
function ArcIO:workStr( dir, name )
	-- If dir is empty then don't use ... todo, just handle this properly 
	-- Create dir if it doesn't already exist 
	work_output_dir = self.output_dir .. dir 
	if not os.execute(string.format("mkdir -p %s", work_output_dir)) then
		error("Error making %s", work_output_dir)
	end
	work_str = work_output_dir .. "/" .. name 
	return work_str
end

-- Load torch data struct 
function ArcIO:loadTorch( dir, name )
	data_fname = self:workStr( dir, name .. '.t7' )
	print("Loading: " .. data_fname )

	return torch.load(data_fname)
end

-- Dump torch data structure 
function ArcIO:dumpTorch( data, dir, name )
	if self.scan_num == nil then
		print("scan_num == nil, gotta specify that yo")
		return
	end
		
	data_fname = self:workStr( dir, name .. '.t7' )
	print("Saving: " .. data_fname )

	-- Fill out output data structure, must have scan_num and job_id  
	data.scan_num = self.scan_num
	data.job_id  = self.job_id 
	data.work_id = self.work_id
	torch.save(data_fname, data)
end

-- Dump image data 
function ArcIO:dumpImage( im, dir, name )
	image_fname = self:workStr( dir, name .. '.jpg' )
	print("Saving: " .. image_fname )
	image.save(image_fname, im)
end





