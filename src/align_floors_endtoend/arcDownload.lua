local path = require 'path'
local fs = require 'fs'
local io = require 'io'
local pcl = PointCloud.PointCloud
local log = require '../util/log'
require '../data/Arc.lua'

local arcDownload = Class()

function arcDownload:__init(scanid, version)
	if scanid then
		self.scanid = scanid
	else
		error('arcDownload needs scanid')
	end
	if version then
		self.version = version
	else
		error("version needed")
	end
end

function arcDownload:makeSourceDir(arc)
	sourcedir = self:getSourceDir(arc)
	util.fs.mkdir_p(sourcedir)
	return sourcedir
end

function arcDownload:getSourceDir(arc)
	return path.join(arc:dirname(),'source', 'faro')
end

function arcDownload:getImgDir(arc)
	return path.join(arc:dirname(), 'work', self.version, 'flattened')
end

function arcDownload:getImgArc(arc)
	return arc.work[self.version].flattened;
end

function arcDownload:getCombinedDir(arc)
	return path.join(arc:dirname(), 'work', self.version, 'combined')
end

function arcDownload:getPCLDir(arc)
	return path.join(arc:dirname(), 'work', self.version, 'pointcloud')
end

function arcDownload:makeWorkDirs(arc)
	local dirname = arc:dirname()
	local pcldir, imgdir
	local pcldir = self:getPCLDir(arc)
	util.fs.mkdir_p(pcldir)
	local imgdir = getImgDir('arc')
	util.fs.mkdir_p(imgdir)
	return pcldir,imgdir	
end

function arcDownload:copy_file(from, to)
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

function arcDownload:flattenedToTransformation()
	if self.scanid then		
		local function mkDestDir(arc)
			local params = {}
			util.fs.mkdir_p(self:getCombinedDir(arc))
			params["from"] = self:getImgDir(arc)
			params["fromArc"] = self:getImgArc(arc)
			params["combined"] = self:getCombinedDir(arc)
			print(params["from"])
			print(params["fromArc"])
			print(params["combined"])

			return params
		end

		local function doForPair(fname1, fname2, params)
			local bname1 = path.basename(fname1)	
			local bname2 = path.basename(fname2)
			local bname1 = string.sub(bname1,1,#bname1-4)
			local bname2 = string.sub(bname2,1,#bname2-4)
			local bname1 = string.sub(bname1,#bname1-2,#bname1)
			local bname2 = string.sub(bname2,#bname2-2,#bname2)
			
			bestT,trans1, trans2, combined, inliers, src_cnt_h, src_cnt_w, tgt_cnt_h, tgt_cnt_w = geom.FloorTransformation.findTransformationOurs(fname1,fname2)
			
			collectgarbage()
			
			local all_scores = torch.Tensor(table.getn(combined),9)
			
			for i = 1,table.getn(combined) do
			
				local comb = combined[i]
				local sch = src_cnt_h[i]
				local scw = src_cnt_w[i]
				local tch = tgt_cnt_h[i]
				local tcw = tgt_cnt_w[i]
				local validation_scores = align_floors_endtoend.validation.compute_score(comb,sch,scw,tch,tcw)
				all_scores[i] = validation_scores
			end
			
			local sorted_scores, order_scores = all_scores:sort(1)
			
			local order = order_scores:transpose(1,2)[1]
			
			local i = 1
			local k = 1
			while k < 6 and i <= table.getn(combined) do
				
				local total_score = math.ceil(all_scores[order[i]][1] * 100)
				local vrs_no_cap  = math.ceil(all_scores[order[i]][2] * 100)
				local vrs_capped  = math.ceil(all_scores[order[i]][3] * 100)
				local vid_fine_t  = math.ceil(all_scores[order[i]][4] * 100)
				local vid_rugh_t  = math.ceil(all_scores[order[i]][7] * 100)
				
				collectgarbage()
				
				if total_score <= 50 and vid_fine_t <= 85 and vid_rugh_t <= 65 and vrs_capped <= 25 then
				
					local cname = path.join(params["combined"],bname1..'_'..bname2..'_'..total_score..'_'..vrs_no_cap..'_'..vrs_capped..'_'..vid_fine_t..'_'..vid_rugh_t..'_'..i..'_'..inliers[i]..'.png')
					image.save(cname, combined[i])
					
					k = k + 1
					
				end
				
				i = i + 1
			end
		end
		extension = '.png'
		self:doForEveryPairInArc(mkDestDir, doForPair, extension, true)
	end
end		


function arcDownload:doForEveryPairInArc(getSourceDestInfo, doForPair, extension, download)
	local arc = data.Arc.get(self.scanid, 
		function(err,arc) 
			if err then
				print("logging error")
				log.error(err)
			else
				params=getSourceDestInfo(arc)
				if(download) then
					sourceArc = params["fromArc"];
					i=1
					newArray = {}
					for key,value in pairs(sourceArc) do 
						if util.fs.extname(key) == extension then
							print(i)
							newArray[i]=sourceArc[key]
							i = i +1
						end
					end

					local function recursiveDownload(fileArray, current, max, callback)
						if(current == max) then
							callback()
						else
							fileArray[current]:download(function (err, filename)
								if err then
									log.error(err)
								end
								recursiveDownload(fileArray, current+1, max, callback)
								end)
						end
					end
					
					local counter = 0;
					local function nextFunction(max, callback)
						counter = counter +1;
						if(counter == max) then
							callback()
						end
					end

					local function recursiveDownloadAsync(fileArray, current, max, callback)
						if not(current == max) then
							fileArray[current]:download(function (err, filename)
								if err then
									log.error(err)
								end
								recursiveDownload(fileArray, current+1, max, callback)
								nextFunction(max, callback)
								end)
						end
					end

					local function postDownload()
						local filetb = util.fs.files_only(params["from"])
						for i=1,#filetb do
							local fname1 = filetb[i]
							if util.fs.extname(fname1) == extension then
								for j=i+1, math.min(#filetb, i+1) do
									local fname2 = filetb[j]	
									if util.fs.extname(fname2) == extension then
										doForPair(fname1, fname2, params)
										collectgarbage()
									end
								end
							end
						end
						print('saving arc')
						--arc:save()			
					end
					recursiveDownloadAsync(newArray, 1, i, postDownload)
					
				end
				
				
				collectgarbage()
			end
			

			end)
collectgarbage()
end

