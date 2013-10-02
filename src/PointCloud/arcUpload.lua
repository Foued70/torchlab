local path = require 'path'
local fs = require 'fs'
local io = require 'io'
local pcl = PointCloud.PointCloud
local log = require '../util/log'

local arcUpload = Class()

function arcUpload:__init(scanid)
	if scanid then
		self.scanid = scanid
	else
		error('arcUpload needs scanid')
	end
end

local function makeSourceDir(arc)
	local dirname = arc:dirname()
	local sourcedir = path.join(dirname,'source')
	if not arc.source then
		util.fs.mkdir_p(sourcedir)
		sourcedir = path.join(sourcedir,'faro')
		util.fs.mkdir_p(sourcedir)
	else
		sourcedir = path.join(sourcedir,'faro')
		if not arc.source.faro then
			util.fs.mkdir_p(sourcedir)
		end
	end
	return sourcedir
end

local function makeWorkDirs(arc,version)
	local dirname = arc:dirname()
	local pcldir, imgdir
	local sourcedir = makeSourceDir(arc)
	-- check if work dir, version dir and pointcloud dir exist
	local workdir = path.join(dirname, 'work')
	util.fs.mkdir_p(workdir)
	local verdir = path.join(workdir, version)
	util.fs.mkdir_p(verdir)
	local pcldir = path.join(verdir, 'pointcloud')
	util.fs.mkdir_p(pcldir)
	local imgdir = path.join(verdir, 'flattened')
	util.fs.mkdir_p(imgdir)
	return sourcedir,pcldir,imgdir	
end

local function copy_file(from, to)
	local file_i = io.open(from, 'r')
	local file_o = io.open(to, 'w')
	local strg = file_i:read()
	while strg and (strg:len() > 0) do
		file_o:write(strg)
		strg = file_i:read()
	end
	file_i:close()
	file_o:close()
end

local function saveODAndFlatten(xyz,odname,imgname, scale)
	local p = pcl.new(xyz,10)
	p:make_flattened_images(scale)
	p:write(odname)
	print('saved '..odname)
	image.save(imgname, p.imagez)
	print('saved '..imgname)
	p=nil
end

function arcUpload:uploadSourceXYZ(XYZdir)
	if self.scanid and XYZdir and util.fs.is_dir(XYZdir) then
	
		local arc = data.Arc.get(self.scanid, 
					function(err,arc) 
						if err then
							log.error(err)
						else
							local sourcedir = makeSourceDir(arc)
							-- copy files in XYZdir to arc
							local filetb = util.fs.files_only(XYZdir)
							for i=1,#filetb do
								local fname = filetb[i]
								if util.fs.extname(fname) == '.xyz' then
									local oname = path.join(sourcedir,path.basename(fname))
									copy_file(fname,oname)
									print('copied '..fname..' to '..oname)
								end
							end
						end
						print('saving arc')
						arc:import_all()
						arc:save()
					end)
	end
	collectgarbage()				
end

function arcUpload:save()

	local arc = data.Arc.get(self.scanid, 
					function(err,arc) 
						if err then
							print('logging error')
							log.error(err)
						end
						print('saving arc')
						arc:save()
					end)
	collectgarbage()				
	
end

function arcUpload:uploadODAndFlatFiles(version,scale)
	if self.scanid then
					
		local arc = data.Arc.get(self.scanid, 
					function(err,arc) 
						if err then
							log.error(err)
						else
							-- for each XYZFile in sourcedir, load as PointCloud and save as ODFile
							sourcedir,pcldir,imgdir=makeWorkDirs(arc,version)
							local filetb = util.fs.files_only(sourcedir)
							for i=1,#filetb do
								local fname = filetb[i]
								if util.fs.extname(fname) == '.xyz' then
									local bname = path.basename(fname)
									local oname = path.join(pcldir,string.sub(bname,1,#bname-4)..'.od')
									local iname = path.join(imgdir,string.sub(bname,1,#bname-4)..'.png')
									saveODAndFlatten(fname,oname,iname, scale)
								end
								collectgarbage()
							end
							collectgarbage()
						end
						print('saving arc')
						arc:import_all()
						arc:save()
					end)
	end	
	collectgarbage()					
end

function arcUpload:uploadSourceAndWork(XYZdir,scale)
	version='a_00'
	
	if self.scanid and XYZdir and util.fs.is_dir(XYZdir) then
	
		local arc = data.Arc.get(self.scanid, 
					function(err,arc) 
						if err then
							log.error(err)
						end
					end)
		
		local arc = data.Arc.get(self.scanid, 
					function(err,arc) 
						if err then
							log.error(err)
						else
							sourcedir,pcldir,imgdir=makeWorkDirs(arc,version)
							local filetb = util.fs.files_only(XYZdir)
							for i=1,#filetb do
								local fname = filetb[i]
								if util.fs.extname(fname) == '.xyz' then
									local oname = path.join(sourcedir,path.basename(fname))
									copy_file(fname,oname)
									print('copied '..fname..' to '..oname)
								end
								collectgarbage()
							end
							collectgarbage()
							filetb = util.fs.files_only(sourcedir)
							for i=1,#filetb do
								local fname = filetb[i]
								if util.fs.extname(fname) == '.xyz' then
									local bname = path.basename(fname)
									local oname = path.join(pcldir,string.sub(bname,1,#bname-4)..'.od')
									local iname = path.join(imgdir,string.sub(bname,1,#bname-4)..'.png')
									saveODAndFlatten(fname,oname,iname, scale)
								end
								collectgarbage()
							end
							collectgarbage()
						end
						print('saving arc')
						arc:import_all()
						arc:save()
					end)
	end	
	collectgarbage()
end		