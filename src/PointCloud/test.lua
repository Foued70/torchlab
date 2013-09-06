require '../image/init'
arcUpload = PointCloud.arcUpload
--scanid = 'motor-unicorn-0776'
scanid = 'virtuous-walk-1066'
--scanid = 'sleepy-art-2566'
--xyzdir = '/Users/lihui815/newdir/2013-08-08_730_5th_Avenue'
version = 'a_00'
scale = 0.015
local path = require 'path'
local io = require 'io'

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
local odir_f = '/Users/lihui815/tmp3/flat'
local odir_c = '/Users/lihui815/tmp3/corners'
local odir = '/Users/lihui815/tmp3'
	
--[[]]
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
	if path.basename(fname) ~= '.DS_Store' then
	--	local poname = path.join(podir,path.basename(string.sub(fname,1,#fname-4))..'.od')
		local ioname_f = path.join(odir_f,path.basename(string.sub(fname,1,#fname-3))..'.png')
		local ioname_c = path.join(odir_c,path.basename(string.sub(fname,1,#fname-3))..'.dat')
		local ioname_p = path.join(odir,path.basename(string.sub(fname,1,#fname-3))..'.png')
	--	local ioname = path.join(odir,path.basename(string.sub(fname,1,#fname-3))..'.png')
	--	local oname = path.join(odir,path.basename(string.sub(fname,1,#fname-3))..'.xyz')
		
		print('loading '..fname)
		
	--	local a = pcl.new(fname,range,nil)
		local a = pcl.new(fname)
		fname = nil
		print(a.count)
		
		loadtime = log.toc()
		log.tic()
	
	--	local b = a:find_connections()	
		local d,corn = a:make_flattened_images(scale,nil,30)
	--[[]]
		local c = d:clone():cdiv(d:clone():add(0.000000001))
		d:mul(2):add(c)
		d:div(d:max()+0.0000000001)
	--[[]]
		a = nil
		
		downsampletime = log.toc()
		log.tic()
	
	--	image.save(ioname_p,b)
		image.save(ioname_f,d)
		torch.save(ioname_c,corn)
	
		savetime = log.toc()
	
		b = nil
		c = nil
		d = nil
		corn = nil
	
		collectgarbage()
	
		print('total time: '..(loadtime+flattentime+downsampletime+savetime)..', lt: '..loadtime..', ft: '..flattentime..', st: '..savetime..', dt: '..downsampletime)
		print()
	end
end	
--[[]]

--[[]]
indir = '/Users/lihui815/tmp3/flat'
crdir = '/Users/lihui815/tmp3/corners'
otdir = '/Users/lihui815/tmp3/combined'

filestb = util.fs.files_only(indir)
print(#filestb)
local mm = 6
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
		--local bname1 = string.sub(bname1,#bname1-2,#bname1)
		--local bname2 = string.sub(bname2,#bname2-2,#bname2)
		
		cornm1 = path.join(crdir,bname1..'.dat')
		cornm2 = path.join(crdir,bname2..'.dat')
		
		print(bname1..' - '..bname2..': '..inl:max()..' '..inl:min()..' '..sorted_scores:max()..' '..sorted_scores:min())
	
		--align_floors_endtoend.FloorTransformation.findTransformationSavedCorners(fname1,fname2,cornm1,cornm2,true)
		
		--[[]]
		local bestT,trans1, trans2, combined, inliers, anglediff, src_cnt_h, src_cnt_w, tgt_cnt_h, tgt_cnt_w, size_x_all, size_y_all, scores = align_floors_endtoend.FloorTransformation.findTransformationSavedCorners(fname1,fname2,cornm1,cornm2)
		collectgarbage()
	    
		local all_scores = scores:clone():mul(-1)
		local numcandidates = all_scores:nElement()
		local sorted_scores, order = all_scores:sort()	
			
		--print(sorted_scores)
		local inl = torch.Tensor(inliers)
			
		local i = 1
		local k = 1
		while k < 6 and i <= numcandidates do
			
			local j = order[i]
			
			local total_score = all_scores[j]
			local innum = inliers[j]
			local angnum = anglediff[j]
				
			collectgarbage()
			
			if total_score < -1 then
			--ground_truth_score = align_floors_endtoend.FloorTransformation.scoreTransformationPair(bestT[j].H, truthH, size_x_all[j], size_y_all[j])
			ground_truth_score = 0
					
			local cname = path.join(otdir,bname1..'_'..bname2..'_'..i..'_'..j..'_'..total_score..'_'..innum..'_'..angnum..'_'..'truth'..ground_truth_score..'.png')
			image.save(cname, combined[j])
						
			k = k + 1
			end
			i = i + 1
		end
		--[[]]
	end
end
--[[]]