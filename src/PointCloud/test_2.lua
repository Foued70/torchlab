require '../image/init'
scale = 0.01
local path = require 'path'
local io = require 'io'

local log = require '../util/log'
local path = require 'path'
local fs = require 'fs'
local pcl = PointCloud.pointcloud
local Homography = geom.Homography
local FloorTransformation = align_floors_endtoend.FloorTransformation

local range = 10
local nl = 8

local tdir = '/Users/lihui815/tmp2/2013_09_08_Office'
local sdir = path.join(tdir,'OD')

local odir_o = path.join(tdir,'OD')
local odir_f = path.join(tdir,'FLAT')
local odir_c = path.join(tdir,'CORNERS')
local odir_a = path.join(tdir,'COMBINED')
local odir_s = path.join(tdir,'STITCHED')
local odir_t = path.join(tdir,'TRANSFORM')
local odir_ft = path.join(tdir,'FULLTRANS')
local odir_ct = path.join(tdir,'CORNERSSTITCH')

local xyzext = '.xyz'
local oddext = '.od'
local datext = '.dat'
local imgext = '.png'

--local pclext=xyzext
local pclext=oddext

local sweeps = torch.Tensor({1000,1001,1002,1003,1004,1005,
							 1006,1007,1008,1009,1010,1011,
							 1012,1013,1014,1015,1016,1017,
							 1018,1019,1020,1021,1022,1023,
							 1024,1025,1026,1027,1028,1029,
							 1030,1031,1032,1033,1034,1035,
							 1036,1037,1038,1039,1040,1041,
							 1042,1043,1044,1045,1046,1047,
							 1048,1049,1050,1051,1052,1053,
							 1054,1055,1056,1057,1058,1059,
							 1060,1061,1062,1063,1064,1065,
							 1066,1067,1068,1069,1070,1071,
							 1072,1073,1074,1075,1076,1077,
							 1078,1079,1080,1081,1082,1083,
							 1084,1085,1086,1087,1088,1089,
							 1090,1091,1092,1093,1094,1095,
							 1096,1097,1098,1099,1100,1101,
							 1102,1103,1104})


--[[]]
local sweep_pairs = sweeps:clone():repeatTensor(2,1):transpose(1,2)
sweep_pairs:select(2,2):mul(0)
sweep_pairs:select(2,2):sub(1,sweep_pairs:size(1)-1):add(sweep_pairs:select(2,1):sub(2,sweep_pairs:size(1)))
sweep_pairs:select(2,2):sub(sweep_pairs:size(1)-0,sweep_pairs:size(1)):add(sweep_pairs:select(2,1):sub(1,1))

--[[]]
local sweep_pairs_new = torch.zeros(sweep_pairs:size())
sweep_pairs_new:sub(1,sweep_pairs:size(1)-30):add(sweep_pairs:sub(1+30,sweep_pairs:size(1)))
sweep_pairs_new:sub(sweep_pairs:size(1)-30+1,sweep_pairs:size(1)):add(sweep_pairs:sub(1,30))

sweep_pairs=sweep_pairs_new

--[[

local sweep_pairs_new = torch.zeros(sweep_pairs:size())
sweep_pairs_new:select(2,1):add(sweep_pairs:select(2,2))
sweep_pairs_new:select(2,2):add(sweep_pairs:select(2,1))
sweep_pairs = sweep_pairs_new

--[[
local sweep_pairs = torch.Tensor({{1002,1005},
                                  {1005,1007},
                                  {1007,1011},
                                  {1011,1013},
                                  {1013,1015},
                                  {1015,1017},
                                  {1017,1021},
                                  {1021,1024},
                                  {1024,1027},
                                  {1027,1029},
                                  {1029,1030},
                                  {1030,1032},
                            	  {1032,1035},
 	                              {1035,1039},
                                  {1039,1042},
                                  {1042,1045},
                                  {1045,1048},
                                  {1048,1051},
                                  {1051,1053},
                                  {1053,1056},
                                  {1056,1057},
                                  {1057,1060},
                                  {1060,1062},
                                  {1062,1064},
                                  {1064,1068},
                                  {1068,1072},
                                  {1072,1075},
                                  {1075,1079},
                                  {1079,1082},
                                  {1082,1086},
                                  {1086,1088},
                                  {1088,1091},
                                  {1091,1095},
                                  {1095,1097},
                                  {1097,1099},
                                  {1099,1100},
                                  {1100,1102},
                                  {1102,1104},
                                  {1104,1002}
                                  })
--[[]]


--[[
local loadtime = 0
local flattentime = 0
local savetime = 0

for j=1,sweeps:size(1) do
	
	log.tic()
	
	local bname = ''..sweeps[j]
	local fname = path.join(sdir,bname..pclext)
	
	local ioname_f = path.join(odir_f,bname..imgext)
	local ioname_c = path.join(odir_c,bname..datext)
	local ioname_o = path.join(odir_o,bname..oddext)
		
	print('loading '..fname)
		
	local a = pcl.new(fname)
	fname = nil
		
	loadtime = log.toc()
	
	--a:axis_align()
	--a:make_normal_map(true,true)
	
	log.tic()
	
	local b,c = a:make_flattened_images(scale,nil,30)
		
	local bn = b:clone():cdiv(b:clone():add(0.000000001))
	b:mul(4):add(bn)
	b:div(b:max()+0.0000000001)
	
	flattentime = log.toc()
	
	log.tic()
	
	image.save(ioname_f,b)
	torch.save(ioname_c,c)
	a:write(ioname_o)
		
	a = nil
	
	savetime = log.toc()
	
	b = nil
	c = nil
	bn = nil
	
	collectgarbage()
	
	print('total time: '..(loadtime+flattentime+savetime)..', lt: '..loadtime..', ft: '..flattentime..', st: '..savetime)
	print()
	collectgarbage()
end	

--[[

for ii=1,1 do --sweep_pairs:size(1) do
	
	local bname1 = ''..sweep_pairs[ii][1]
	local bname2 = ''..sweep_pairs[ii][2]
	local fname1 = path.join(odir_f,bname1..imgext)
	local fname2 = path.join(odir_f,bname2..imgext)
		
	cornm1 = path.join(odir_c,bname1..datext)
	cornm2 = path.join(odir_c,bname2..datext)
		
	print(bname1..' - '..bname2)
	
	FloorTransformation.findTransformationSavedCorners(fname2,fname1,cornm2,cornm1,true)
	
end

--[[

local find_cand_time = 0
local filter_cand_time = 0
local img_dest

for ii=1,sweep_pairs:size(1) do
	
	local bname1 = ''..sweep_pairs[ii][1]
	local bname2 = ''..sweep_pairs[ii][2]
	local fname1 = path.join(odir_f,bname1..imgext)
	local fname2 = path.join(odir_f,bname2..imgext)
		
	cornm1 = path.join(odir_c,bname1..datext)
	cornm2 = path.join(odir_c,bname2..datext)
		
	print(bname1..' - '..bname2)
	
	log.tic()
	local bestT,trans2, trans1, combined, inliers, anglediff, tgt_cnt_h, tgt_cnt_w, src_cnt_h, src_cnt_w, size_x_all, size_y_all, scores = FloorTransformation.findTransformationSavedCorners(fname2,fname1,cornm2,cornm1)
	
	find_cand_time = log.toc()
	
	log.tic()
	
	collectgarbage()
	    
	local all_scores = scores:clone():mul(-1)
	local numcandidates = all_scores:nElement()	
	
	local sorted_scores = all_scores
	local order = torch.range(1,numcandidates)
			
	local inl = torch.Tensor(inliers)
		
	print(inl:max()..' '..inl:min()..' '..sorted_scores:max()..' '..sorted_scores:min())
			
	local i = 1
	local k = 1
	while k < 2 and i <= numcandidates do
			
		local j = order[i]
			
		local total_score = all_scores[j]
		local innum = inliers[j]
		local angnum = anglediff[j]
				
		collectgarbage()
			
		if total_score < -0 then
			ground_truth_score = 0
			local cname = path.join(odir_a,bname1..'_'..bname2..'_'..i..'_'..j..'_'..total_score..'_'..innum..'_'..angnum..'_'..'truth'..ground_truth_score..'.png')
			image.save(cname, combined[j])
			torch.save(path.join(odir_t,bname1..'_'..bname2..'.dat'),bestT[j].H)
						
			k = k + 1
		end
		collectgarbage()
		i = i + 1
	end
	filter_cand_time = log.toc()
	
	collectgarbage()
	print('total time: '..(find_cand_time+filter_cand_time)..', find: '..find_cand_time..', filter: '..filter_cand_time)
	print()
	
end

--[[

local sizeH = 5000
local sizeW = 4000
local find_cand_time = 0
local filter_cand_time = 0
local translateH=2000
local translateW=600

for ii=5,sweep_pairs:size(1) do
	
	local bname1 = ''..sweep_pairs[ii][1]
	local bname2 = ''..sweep_pairs[ii][2]
	local fname1 = path.join(odir_f,bname1..imgext)
	local fname2 = path.join(odir_f,bname2..imgext)
		
	cornm1 = path.join(odir_c,bname1..datext)
	cornm2 = path.join(odir_c,bname2..datext)
	
	print(bname1..' - '..bname2)
	
	if ii == 1 then
		
		print('set up first pass')
		local img = opencv.Mat.new(fname1)
		img:convert("RGB2GRAY");

		local trans_trans = Homography.new(0,torch.Tensor({translateH,translateW}))
		local src_transform = opencv.Mat.new(trans_trans:getEquivalentCV())
    	img  =  opencv.imgproc.warpImage(img, src_transform, sizeW, sizeH)
		local img_tens = img:toTensor():type('torch.DoubleTensor')
		img_tens:div(img_tens:max()+0.000000001)

		image.save(path.join(odir_s,'000.png'),img_tens)
		
		local corners = torch.load(cornm1)
		local corners_tmp = torch.zeros(3,corners:size(1))
		corners_tmp:select(1,1):add(corners:select(2,1))
		corners_tmp:select(1,2):add(corners:select(2,2))
		corners_tmp:select(1,3):fill(1)
		corners_tmp = trans_trans.H * corners_tmp
		corners = torch.zeros(corners:size())
		corners:select(2,1):add(corners_tmp:select(1,1))
		corners:select(2,2):add(corners_tmp:select(1,2))
		
		print('corners: '..corners:size(1))
		
		torch.save(path.join(odir_ct,bname1..datext),corners)
		
		img = nil
		trans_trans = nil
		src_transform = nil	
		corners = nil
		corners_tmp = nil
		
	end
	
	collectgarbage()
	
	local m = ''..(ii - 1)
	while #m < 3 do
		m = '0'..m
	end
	
	fname1 = path.join(odir_s,m..'.png')
	cornm1 = path.join(odir_ct,bname1..datext)
	
	print('updated fnames')
	
	log.tic()
	local bestT,trans2, trans1, combined, inliers, anglediff, tgt_cnt_h, tgt_cnt_w, src_cnt_h, src_cnt_w, size_x_all, size_y_all, scores = FloorTransformation.findTransformationSavedCorners(fname2,fname1,cornm2,cornm1)
	
	find_cand_time = log.toc()
	
	log.tic()
	
	collectgarbage()
	    
	local all_scores = scores:clone():mul(-1)
	local numcandidates = all_scores:nElement()	
	
	local sorted_scores = all_scores
	local order = torch.range(1,numcandidates)
			
	local inl = torch.Tensor(inliers)
	
	if inliers ~= nil and #inliers > 0 then
	print(inl:max()..' '..inl:min()..' '..sorted_scores:max()..' '..sorted_scores:min())
			
	local i = 1
	local k = 1
	while k < 2 and i <= numcandidates do
			
		local j = order[i]
			
		local total_score = all_scores[j]
		local innum = inliers[j]
		local angnum = anglediff[j]
				
		collectgarbage()
			
		if total_score < -0 then
		
			print('found something')
			ground_truth_score = 0
			local cname = path.join(odir_a,bname1..'_'..bname2..'_'..i..'_'..j..'_'..total_score..'_'..innum..'_'..angnum..'_'..'truth'..ground_truth_score..'.png')
			image.save(cname, combined[j])
			torch.save(path.join(odir_t,bname1..'_'..bname2..'.dat'),bestT[j].H)
			
			local trans_trans = bestT[j]
		
			local corners = torch.load(cornm2)
			local corners_tmp = torch.zeros(3,corners:size(1))
			corners_tmp:select(1,1):add(corners:select(2,1))
			corners_tmp:select(1,2):add(corners:select(2,2))
			corners_tmp:select(1,3):fill(1)
			corners_tmp = trans_trans.H * corners_tmp
			corners = torch.zeros(corners:size())
			corners:select(2,1):add(corners_tmp:select(1,1))
			corners:select(2,2):add(corners_tmp:select(1,2))
			--corners = torch.cat(torch.load(cornm1),1)
			
			print('corners: '..corners:size(1))
		
			torch.save(path.join(odir_ct,bname2..datext),corners)
			torch.save(path.join(odir_ft,bname1..'_'..bname2..'.dat'),trans_trans.H)
			
			local combined_tens = combined[j]:max(1):squeeze():repeatTensor(3,1,1)
			combined_tens:type('torch.DoubleTensor')
			combined_tens:div(combined_tens:max()+0.00000000001)
			
			m = ''..(ii)
			while #m < 3 do
				m = '0'..m
			end
			image.save(path.join(odir_s,m..'.png'),combined_tens)
		
			img = nil
			trans_trans = nil
			src_transform = nil	
			combined_tens = nil
			corners = nil
			corners_tmp = nil
			
			print('saved everything')
						
			k = k + 1
		end
		collectgarbage()
		i = i + 1
	end
	end
	filter_cand_time = log.toc()
	
	collectgarbage()
	print('total time: '..(find_cand_time+filter_cand_time)..', find: '..find_cand_time..', filter: '..filter_cand_time)
	print()
	
end

--[[]]

local sizeH = 5000
local sizeW = 4000
local translateH=2000
local translateW=600
local currTrans1 = Homography.new(0,torch.Tensor({0,0}))
currTrans1.H = torch.load(path.join(odir_ft,'1101_1102.dat'))

--local currTrans2 = Homography.new(0,torch.Tensor({0,0}))
--currTrans2.H = torch.load(path.join(odir_ft,'1100_1099.dat'))

--local tns_tmp = torch.zeros(3,sizeH,sizeW)
--image.save(path.join(odir_s,'000.png'),tns_tmp)

local cnt

for i=73,sweep_pairs:size(1) do

	ii = i
	cnt = ii
	
	log.tic()
	local bname1 = ''..sweep_pairs[ii][1]
	local bname2 = ''..sweep_pairs[ii][2]

	local transform = Homography.new(0,torch.Tensor({0,0}))
	transform.H = torch.load(path.join(odir_t,bname1..'_'..bname2..'.dat'))
	
	currTrans1 = currTrans1:combineWith(transform)
	
	local trans_trans = Homography.new(0,torch.Tensor({translateH,translateW}))
	trans_trans = trans_trans:combineWith(currTrans1)
	
	local img_src = opencv.Mat.new(path.join(odir_f,bname2..'.png'))
	img_src:convert("RGB2GRAY");
	
	local src_transform = opencv.Mat.new(trans_trans:getEquivalentCV())
    local warped_src  =  opencv.imgproc.warpImage(img_src, src_transform, sizeW, sizeH)
    
    local k = ''..(cnt-1)
	while #k < 3 do
		k = '0'..k
	end
	
	local img_dest = opencv.Mat.new(path.join(odir_s,k..'.png'))
	img_dest:convert("RGB2GRAY");
    
    local combined = torch.zeros(3,sizeH,sizeW)
    combined:select(1,1):add(img_dest:toTensor():type('torch.DoubleTensor'))
    combined:select(1,2):add(warped_src:toTensor():type('torch.DoubleTensor'))
    combined:div(combined:max()+0.00000001)
    combined = combined:max(1):squeeze():repeatTensor(3,1,1)
    
	local k = ''..(cnt)
	while #k < 3 do
		k = '0'..k
	end
	
	image.save(path.join(odir_s,k..'.png'),combined)
	torch.save(path.join(odir_ft,bname1..'_'..bname2..'.dat'),currTrans1.H)
	print(ii..' '..bname1..' '..bname2..' '..log.toc())
	
	k = nil
	bname1 = nil
	bname2 = nil
	img_dest = nil
	img_src = nil
	warped_src = nil
	combined = nil
	transform = nil
	trans_trans = nil
	
	collectgarbage()

end

--[[]]