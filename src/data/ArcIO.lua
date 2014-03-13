--[[
	ArcIO:
		Simple utility for reading and writing Arc data
		- Currently there is no magic, but eventually I imagine that
		  one could specify the current job_id and work_id they are on
		- Also it could be cool to have specs for different types of work 
		  that can be loaded and make ArcIO more automatic 
]]--
ArcSpecs = data.ArcSpecs

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
	self.pointcloudt7_path = self.job_path .. '/work/po_scan/'
	if work_id then 
		self.output_dir = self.job_path .. '/work/' .. work_id .. "/"
	end
	self.scan_num = nil 

	-- Create pc.t7 data dir if it doesn't already exist ... TODO use util.fs. 
	if not os.execute(string.format("mkdir -p %s", self.pointcloudt7_path )) then
		error("Error making %s", self.job_path .. '/work/po_scan')
	end

	self.job_id = job_id
	self.work_id = work_id

	print("pointcloud_path: ", self.pointcloud_path)
	print("pointcloudt7_path: ", self.pointcloudt7_path)
	print("output_dir: ", self.output_dir)
end

-- Shortcut for loading pointcloud scan data 
function ArcIO:getScan( scan_num, force_reload )
	scan_str = string.format('%.3d', scan_num)	
	scant7_fname = self.pointcloudt7_path .. scan_str .. '.t7'	
	-- If forcing re-load, then just load by string, otherwise check if we have already saved a pointcloud in torch fmt	
	if force_reload == true or io.open( scant7_fname ) == nil then
		self.scan_num = scan_num
		scan_fname = self.pointcloud_path .. scan_str
		
		loader = pointcloud.loader.load_pobot_ascii( scan_fname )
		pc = pointcloud.pointcloud.new(loader) 
		print("Saving scan: " .. scant7_fname)
		torch.save(scant7_fname, pc)
	else
		scan_fname = scant7_fname
		pc = torch.load( scan_fname )
	end
	print("Loading scan: ", scan_fname)
	return pc
end

-- Shortcut for loading scan images ... loads png files. 
-- TODO: set up a script to automatically convert .nef to .png 
function ArcIO:getImage( scan_num, im_num )
	im_str = string.format('%.3d/01_%.1d.png', scan_num, im_num)		
	im_fname = self.pointcloud_path .. im_str	
	im = image.load(im_fname)
	return im 
end

-- force repopulate of all scans 
function ArcIO:populateScans() 
	scan_ids = ArcSpecs.scan_ids[self.job_id]
	for _,scan_num in ipairs(scan_ids) do 
		self:getScan(scan_num, true)
	end
end

-- Get string to dir in work 
function ArcIO:workStr( dir, name )
	if self.work_id == nil then 
		error("No work_id specified. Usage: ArcIO.new( job_id, work_id )")
	end
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
	data_fname = self:workStr( dir, name .. '.t7' )
	print("Saving: " .. data_fname )

	-- Fill out output data structure	
	data.job_id  = self.job_id 
	data.work_id = self.work_id
	torch.save(data_fname, data)
end

-- Dump image data 
function ArcIO:dumpImage( im, dir, name )
	image_fname = self:workStr( dir, name .. '.png' )
	print("Saving: " .. image_fname )
	image.save(image_fname, im)
end





