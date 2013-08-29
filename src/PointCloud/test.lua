require '../image/init'
arcUpload = PointCloud.arcUpload
--scanid = 'motor-unicorn-0776'
scanid = 'sleepy-art-2566'
--xyzdir = '/Users/lihui815/newdir/2013-08-08_730_5th_Avenue'
version = 'a_00'
scale = 0.015
local path = require 'path'

--au = arcUpload.new(scanid)
--print('set scanid: '..scanid)
--au:uploadSourceXYZ(xyzdir)
--print('uploaded source files from: '..xyzdir)
--au:uploadODAndFlatFiles(version,scale)
--print('computed and uploaded .od and flattened .png for version '..version..' and scale '..scale)
--au:uploadSourceAndWork(xyzdir,scale)


local log = require '../util/log'
local path = require 'path'
local fs = require 'fs'
local pcl = PointCloud.pointcloud

local range = 10
local nl = 8

local tdir = '/Users/lihui815/cloudlab/build/usr/local/tmp/arcs/'..scanid

--local sdir = path.join(tdir,'source/faro')
local sdir = path.join(tdir,'work/'..version..'/pointcloud')
--local podir = path.join(tdir,'work/'..version..'/pointcloud')
--local iodir = path.join(tdir,'work/'..version..'/flattened')
local odir = '/Users/lihui815/tmp2/flat'
	
--[[
local filestb = util.fs.files_only(sdir)
	
local loadtime = 0
local flattentime = 0
local downsampletime = 0
local savetime = 0

local pantime = 0
local dpttime = 0
local ddmtime = 0


for j=1,#filestb do

	print(j)
	
	log.tic()
		
	local fname = filestb[j]
--	local poname = path.join(podir,path.basename(string.sub(fname,1,#fname-4))..'.od')
	local ioname = path.join(odir,'flattenz_'..path.basename(string.sub(fname,1,#fname-3))..'.png')
--	local ioname = path.join(odir,path.basename(string.sub(fname,1,#fname-3))..'.png')
--	local oname = path.join(odir,path.basename(string.sub(fname,1,#fname-3))..'.xyz')
		
	print('loading '..fname)
		
	--local a = pcl.new(fname,range,nil)
	local a = pcl.new(fname)
	fname = nil
	print(a.count)
		
	loadtime = log.toc()
	log.tic()
	
	--local b = a:make_panoramic_normal_map()
	local b = a:make_flattened_images(scale)
	b:mul(256):floor()
	--local c = a:make_panoramic_depth_map()
	--local d = b:clone():cmul(c)
	
	a = nil
	--b = nil
	--c = nil
		
	downsampletime = log.toc()
	log.tic()
	
	image.save(ioname,b)
	
	savetime = log.toc()
	
	b = nil
	
	collectgarbage()
	
	print('total time: '..(loadtime+flattentime+downsampletime+savetime)..', lt: '..loadtime..', ft: '..flattentime..', st: '..savetime..', dt: '..downsampletime)
	print()
end	
--[[]]

--[[]]
indir = '/Users/lihui815/tmp2/flat'
otdir = '/Users/lihui815/tmp2/combined'

filestb = util.fs.files_only(indir)
print(#filestb)
for ii=1,#filestb do
	if not (path.basename(filestb[ii]) == '.DS_Store') then
		jj = ii + 1
		if jj > #filestb then
			jj = 1
		end
		if path.basename(filestb[jj]) == '.DS_Store' then
			jj = jj + 1
			if jj > #filestb then
				jj = 1
			end
		end
	
		fname1 = filestb[ii]
		fname2 = filestb[jj]
	
		local bname1 = path.basename(fname1)	
		local bname2 = path.basename(fname2)
		local bname1 = string.sub(bname1,1,#bname1-4)
		local bname2 = string.sub(bname2,1,#bname2-4)
		local bname1 = string.sub(bname1,#bname1-2,#bname1)
		local bname2 = string.sub(bname2,#bname2-2,#bname2)
	
		bestT,trans1, trans2, combined, inliers, src_cnt_h, src_cnt_w, tgt_cnt_h, tgt_cnt_w = align_floors_endtoend.FloorTransformation.findTransformationOurs(fname1,fname2)
			
		collectgarbage()
			
		local all_scores = torch.Tensor(table.getn(combined),9)
			
		local inl = torch.Tensor(inliers)
		print(fname1..' - '..fname2..': '..inl:max()..' '..inl:min())
		
		local numcandidates = all_scores:size(1)
			
		for i = 1,numcandidates do
			
			local comb = combined[i]
			local sch = src_cnt_h[i]
			local scw = src_cnt_w[i]
			local tch = tgt_cnt_h[i]
			local tcw = tgt_cnt_w[i]
			local combcpy = comb:clone()
				
			combcpy:apply(function(x)
				if x < (0.25 * 255) then
					return 0
				else
					return x
				end
			end)
				
			local validation_scores = align_floors_endtoend.validation.compute_score(combcpy,sch,scw,tch,tcw,bname1..'_'..bname2..'_'..i)
			all_scores[i] = (validation_scores * 10000):ceil()
		end
			
		local sorted_scores, order_scores = all_scores:sort(1)
			
		local order = order_scores:transpose(1,2)[1]
			
		sorted_scores = all_scores:index(1,order)
		print(sorted_scores)
			
		local i = 1
		local k = 1
		while k < 6 and i <= numcandidates do
			
			local j = order[i]
				
			local total_score = all_scores[j][1]
			local vrs_no_cap  = all_scores[j][2]
			local vrs_capped  = all_scores[j][3]
			local vid_fine_t  = all_scores[j][4]
			local vid_rugh_t  = all_scores[j][7]
				
			collectgarbage()
				
			if total_score <= 9000 and vid_fine_t <= 9900 and vid_rugh_t <= 9500 then
				
				local cname = path.join(otdir,bname1..'_'..bname2..'_'..i..'_'..j..'_'..total_score..'_'..vrs_no_cap..'_'..vrs_capped..'_'..vid_fine_t..'_'..vid_rugh_t..'_'..inliers[i]..'.png')
				--local cname = path.join(params["combined"],bname1..'_'..bname2..'_'..j..'_'..total_score..'.png')
				image.save(cname, combined[j])
					
				k = k + 1
					
			else
				print('rejected: '..i..'_'..j..': total: '..total_score..', vid_fine: '..vid_fine_t..', vid_rugh: '..vid_rugh_t)
			end	
				
			i = i + 1
		end
	end
end
--[[]]