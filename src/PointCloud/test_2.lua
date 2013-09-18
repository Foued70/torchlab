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
local odir_fw = path.join(tdir,'TRANSFRWRD')
local odir_bk = path.join(tdir,'TRANSBKWRD')
local odir_ft = path.join(tdir,'TRANSFULL')
local odir_3d = path.join(tdir,'TRANS3D')
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

--[[
local sweep_pairs_new = torch.zeros(sweep_pairs:size())
sweep_pairs_new:sub(1,sweep_pairs:size(1)-30):add(sweep_pairs:sub(1+30,sweep_pairs:size(1)))
sweep_pairs_new:sub(sweep_pairs:size(1)-30+1,sweep_pairs:size(1)):add(sweep_pairs:sub(1,30))

sweep_pairs=sweep_pairs_new

local num = sweep_pairs:size(1)

--[[]]
local loadtime = 0
local flattentime = 0
local savetime = 0

--[[
local a = pcl.new(path.join(sdir,'1000'..pclext))
local rgb = image.load('/Users/lihui815/tmp2/2013_09_08_Office/Images/1000.png'):transpose(1,2):transpose(2,3)
a.rgb=a.rgb:type('torch.DoubleTensor'):div(255)
for i =1,a.count do
	local hw = a.hwindices[i]
	local h = hw[1]
	local w = hw[2]
	a.rgb:sub(i,i):cmul(rgb[h][w])
end
a.rgb = a.rgb:mul(255):floor():type('torch.ByteTensor')
image.display(a:get_rgb_map())
a.format = 0
a:write('/Users/lihui815/1000.xyz')
]]

--[[
local minht = torch.load(path.join(sdir,'1030.od'))[3]:select(2,3):min()/10000
	b = a--:downsample(0.01)
	
	local h = torch.load(path.join(odir_3d, bname..'.dat'))
	local hh = torch.zeros(4,4)
	hh:sub(1,2,1,2):add(h:sub(1,2,1,2))
	hh:sub(1,2,4,4):add(h:sub(1,2,3,3))
	hh[3][4] = minht-b.points:select(2,3):min()
	hh[3][3]=1
	hh[4][4]=1
	torch.save(path.join(odir_3d,'three_d_'..bname..'.dat'),hh)
	--[[
	local pts = torch.ones(4, a.count)
	pts:select(1,1):cmul(b.points:select(2,1))
	pts:select(1,2):cmul(b.points:select(2,2))
	pts:select(1,3):cmul(b.points:select(2,3))
	pts = hh*pts
	local points = torch.zeros(a.points:size())
	points:select(2,1):add(pts:select(1,1))
	points:select(2,2):add(pts:select(1,2))
	points:select(2,3):add(pts:select(1,3))
	b.points = points:mul(10000):floor():div(100)
--[[]]

--[[]]
for j=1,1 do--sweeps:size(1) do
	
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
	
	log.tic()
	
	a:get_normal_map(true)
	local b,c = a:get_flattened_images(scale,nil,30)
		
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

--[[]]

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
	
	if #inliers ~= 0 then
		
		print(inl:max()..' '..inl:min()..' '..sorted_scores:max()..' '..sorted_scores:min())
	end
			
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
	
	local bname2 = ''..sweep_pairs[ii][1]
	local bname1 = ''..sweep_pairs[ii][2]
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
	
	if #inliers ~= 0 then
		
		print(inl:max()..' '..inl:min()..' '..sorted_scores:max()..' '..sorted_scores:min())
	end
			
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

local transfor = Homography.new(0,torch.Tensor({0,0}))
torch.save(path.join(odir_fw, sweep_pairs[1][1]..'.dat'),transfor.H)
local transbak = Homography.new(0,torch.Tensor({0,0}))
torch.save(path.join(odir_bk, sweep_pairs[num][2]..'.dat'),transbak.H)
local pairwise_trans = Homography.new(0,torch.Tensor({0,0}))

for ii=1,num-1 do
	
	local bname1 = ''..sweep_pairs[ii][1]
	local bname2 = ''..sweep_pairs[ii][2]
	
	print(bname1..' - '..bname2)
	
	pairwise_trans.H = torch.load(path.join(odir_t, bname1..'_'..bname2..'.dat'))
	
	transfor = transfor:combineWith(pairwise_trans)
	torch.save(path.join(odir_fw, bname2..'.dat'),transfor.H)
	
	collectgarbage()
	    
	local bname1 = ''..sweep_pairs[num-ii+1][2]
	local bname2 = ''..sweep_pairs[num-ii+1][1]
	
	print(bname1..' - '..bname2)
	
	pairwise_trans.H = torch.load(path.join(odir_t, bname1..'_'..bname2..'.dat'))
	
	transbak = transbak:combineWith(pairwise_trans)
	torch.save(path.join(odir_bk, bname2..'.dat'),transbak.H)
	
	collectgarbage()
	
end

--[[

torch.save(path.join(odir_ft, sweep_pairs[1][1]..'.dat'),Homography.new(0,torch.Tensor({0,0})).H)

for ii=1,num-1 do
	
	local bname = ''..sweep_pairs[ii][2]
	local fname = path.join(odir_f,bname..imgext)
	local imgtens = image.load(fname)
	local corners = torch.Tensor({{              0,              0},
	                              {              0, imgtens:size(2)}})
	
	print(bname)
	
	local transfor = Homography.new(0,torch.Tensor({0,0}))
	local transbak = Homography.new(0,torch.Tensor({0,0}))
	transfor.H = torch.load(path.join(odir_fw, bname..'.dat'))
	transbak.H = torch.load(path.join(odir_bk, bname..'.dat'))
	
	local cornersfor = transfor:applyToPoints(corners:clone())
	local cornersbak = transbak:applyToPoints(corners:clone())
	local cornersavg = cornersfor:clone():mul(num-ii):add(cornersbak:clone():mul(ii)):div(num)
	
	local th = cornersavg[1][1]
	local tw = cornersavg[2][1]
	local costheta = (cornersavg[2][2]-tw)/corners[2][2]
	local sintheta = (th-cornersavg[1][2])/corners[2][2]
	
	local transfull = Homography.new(0,torch.Tensor({0,0}))
	transfull.H = torch.Tensor({{costheta, -sintheta, th},
	                            {sintheta,  costheta, tw},
	                            {       0,         0,  1}})
	
	print(transfor.H)
	print(transbak.H)
	print(transfull.H)
	
	torch.save(path.join(odir_ft, bname..'.dat'),transfull.H)
	
	collectgarbage()
	
end

--[[

local sizeH = 5000
local sizeW = 4000
local translateH=2000
local translateW=600
local currTrans = Homography.new(0,torch.Tensor({0,0}))
local img_dest_tens = torch.zeros(sizeH,sizeW)
--local img_dest_tens = image.load('/Users/lihui815/tmp2/2013_09_08_Office/STITCHED/071_1100.png','torch.DoubleTensor'):select(1,1)
--img_dest_tens:div(img_dest_tens:max())

for ii=1,num do

	log.tic()
	local bname = ''..sweep_pairs[ii][1]

	currTrans.H = torch.load(path.join(odir_ft, bname..'.dat'))
	
	local trans_trans = Homography.new(0,torch.Tensor({translateH,translateW}))
	trans_trans = trans_trans:combineWith(currTrans)
	
	local img_src = opencv.Mat.new(path.join(odir_f,bname..'.png'))
	img_src:convert("RGB2GRAY");
	
	local src_transform = opencv.Mat.new(trans_trans:getEquivalentCV())
    local warped_src_tens  =  opencv.imgproc.warpImage(img_src, src_transform, sizeW, sizeH):toTensor():type('torch.DoubleTensor')
    warped_src_tens:div(warped_src_tens:max()+0.00000001)
    
    local combined = torch.zeros(3,sizeH,sizeW)
    combined:select(1,1):add(img_dest_tens)
    combined:select(1,2):add(warped_src_tens)
    img_dest_tens = combined:max(1):squeeze()
    combined = img_dest_tens:clone():repeatTensor(3,1,1)
   
    local k = ''..(ii)
	while #k < 3 do
		k = '0'..k
	end
	
	image.save(path.join(odir_s,k..'_'..bname..'.png'),combined)
	print(ii..' '..bname..' '..log.toc())
	
	k = nil
	bname = nil
	img_src = nil
	warped_src_tens = nil
	combined = nil
	trans_trans = nil
	
	collectgarbage()

end

--[[

local img_dest_h = 1575
local img_dest_w = 1520

local dest_ch = math.ceil(img_dest_h/2)
local dest_cw = math.ceil(img_dest_w/2)

local currTrans = Homography.new(0,torch.Tensor({0,0}))

for ii=1,num do

	log.tic()
	local bname = ''..sweep_pairs[ii][1]
	print(bname)

	currTrans.H = torch.load(path.join(odir_ft, bname..'.dat'))
	local img_src = image.load(path.join(odir_f,bname..'.png'))
	local img_src_h = img_src:size(2)
	local img_src_w = img_src:size(3)
	local src_ch = math.ceil(img_src_h/2)
	local src_cw = math.ceil(img_src_w/2)
	
	local transl_corner_centered = torch.Tensor({currTrans.H[1][3],currTrans.H[2][3]})
	local src_center = torch.Tensor({src_ch,src_cw})
	local dst_center = torch.Tensor({dest_ch,dest_cw})
	local rot = currTrans.H:sub(1,2,1,2)
	local transl_center_centered = transl_corner_centered - dst_center + (rot * src_center)
	
	print(currTrans.H)
	
	currTrans.H[1][3] = transl_center_centered[1]*scale
	currTrans.H[2][3] = transl_center_centered[2]*scale
	
	torch.save(path.join(odir_3d, bname..'.dat'),currTrans.H)
	
	print(currTrans.H)
	
	collectgarbage()

end

--[[

local sizeH = 5000
local sizeW = 4000
local translateH=2000
local translateW=600
local currTrans1 = Homography.new(0,torch.Tensor({0,0}))
--currTrans1.H = torch.load(path.join(odir_ft,'1101_1102.dat'))

--local currTrans2 = Homography.new(0,torch.Tensor({0,0}))
--currTrans2.H = torch.load(path.join(odir_ft,'1100_1099.dat'))

--local tns_tmp = torch.zeros(3,sizeH,sizeW)
--image.save(path.join(odir_s,'000.png'),tns_tmp)

local cnt

for i=1,sweep_pairs:size(1) do

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