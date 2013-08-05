local hough = image.hough
local wand = image.Wand
local io = require 'io'

--[[]]
local dt = '2013_08_01'
local fnamestart = 127
local fnameend = 135
--[[]]
--[[
local dt = '2013_07_17'
local fnamestart = 70
local fnameend = 89
--[[]]

local dirb = '/Users/lihui815/cloudlab/src/data/test/faro/images/'..dt..'_scans'

local parameters = {
	scale = 0.015,
	radius_max = 5.0,
	center_dist_max = 3.0,

	pixelate_max = 0.20,
	wiggle_max = 0.25,
	vrs_cap = 0.5,
	numangles_vrs = 360,
	numangles_bold = 720,

	perc_of_best_score = 1.15,
	perc_of_best_vrs = 1.25,
	perc_of_best_vid = 1.1,
	
	am_thresh_score = 0.3,			
	am_thresh_vrs_no_cap = 0.05,
	am_thresh_vrs_cap = 0.1,
	am_thresh_vid_rough = 0.40,
	am_thresh_vid_fine = 0.60,

	gd_thresh_score = 0.5,
	gd_thresh_vrs_no_cap = 0.15,
	gd_thresh_vrs_cap = 0.35,
	gd_thresh_vid_rough = 0.725,
	gd_thresh_vid_fine = 0.91,

	mb_thresh_score = 0.6,
	mb_thresh_vrs_no_cap = 0.25,
	mb_thresh_vrs_cap = 0.6,
	mb_thresh_vid_rough = 0.8,
	mb_thresh_vid_fine = 0.97
	}


local function validation_center(ctgt,csrc)
	return ctgt:dist(csrc)
end

local function validation_imgdiff(tns1, tns2, minh, maxh, minw, maxw)
	local tns1cpy = tns1:sub(math.max(1,minh),math.min(tns1:size(1),maxh),math.max(1,minw),math.min(tns1:size(2),maxw)):clone()
						:cdiv(tns1:sub(math.max(1,minh),math.min(tns1:size(1),maxh),math.max(1,minw),math.min(tns1:size(2),maxw))
						+ 0.00000000000000000000000000001)
	local tns2cpy = tns2:sub(math.max(1,minh),math.min(tns1:size(1),maxh),math.max(1,minw),math.min(tns1:size(2),maxw)):clone()
						:cdiv(tns2:sub(math.max(1,minh),math.min(tns1:size(1),maxh),math.max(1,minw),math.min(tns1:size(2),maxw))
						+ 0.00000000000000000000000000001)
	local ovlp = tns1cpy:clone():cmul(tns2cpy):sum()
	--local diff = (tns1cpy-tns2cpy):abs():sum()
	local tnssum = (tns1cpy+tns2cpy):clone()
	local numSum = tnssum:cdiv(tnssum+0.00000000000000000000000000000001):sum()
	--return diff/numSum, diff/(tns1cpy:sum()), diff/(tns2cpy:sum())
	return (1-ovlp/numSum), (1-ovlp/(tns1cpy:sum())), (1-ovlp/(tns2cpy:sum()))
end

local function validation_ray_score(tnstgt, tnssrc, angle, hcnt, wcnt, wiggleroom, cap)

	local h0 = hcnt
	local w0 = wcnt
	local nw = math.cos(angle)
	local nh = math.sin(angle)
	
	local currpt = 2
	local score = 0.0
	
	if not cap then
		cap = math.huge
	end
	
	while(true) do
	
		local currh = nh * currpt + h0
		local currw = nw * currpt + w0
		
		--local inc = 1
		local inc = 1+1/currpt
		
		if currh > tnstgt:size(1) or currw > tnstgt:size(2) or currh < 1 or currw < 1 then
			break
		else
			
			if tnstgt[{currh,currw}] > 0 then
				break
			elseif tnstgt:sub(math.max(currh-wiggleroom,1),
							  math.min(currh+wiggleroom,tnstgt:size(1)),
							  math.max(currw-wiggleroom,1),
							  math.min(currw+wiggleroom,tnstgt:size(2))):max() > 0 then
				score = score + 0
			elseif score > 0 then
				score = score + inc
			elseif tnssrc[{currh,currw}] > 0 then
				score = score + inc
			end
		end
		
		currpt = currpt + 1
		
	end
	
	if score > cap then
		score = cap
	end
	
	return score,math.min(cap,currpt)
end

local function validation_ray(tnstgt, tnssrc, numangles, hcnt, wcnt, wiggleroom, cap)

	if not wiggleroom then
		wiggleroom = 0
	end

	local rettns = torch.zeros(numangles)
	local ang = 0
	local step = 2 * math.pi / numangles
	local multiplier = 0

	rettns:apply(function()
		ang = ang + step
		local ret,r = validation_ray_score(tnstgt, tnssrc, ang, hcnt, wcnt, wiggleroom, cap)
		if r > multiplier then
			multiplier = r
		end
		return ret
	end)
	
	return rettns:sum()/(numangles*multiplier)
end

function create_bold_image_ray(tgt, tgt_bold, angle, hcnt, wcnt, ohcnt, owcnt,rad)

	local h0 = hcnt
	local w0 = wcnt
	local nw = math.cos(angle)
	local nh = math.sin(angle)
	
	local currpt = math.floor(math.max(tgt:size(1),tgt:size(2))/2)+1
	local foundpt = false
	
	while(true) do
	
		local currh = nh * currpt + h0
		local currw = nw * currpt + w0
				
		if not (currh > tgt:size(1) or currw > tgt:size(2) or currh < 1 or currw < 1) then
		
			if math.min(math.pow(currh-hcnt,2)+math.pow(currw-wcnt,2), 
						math.pow(currh-ohcnt,2)+math.pow(currw-owcnt,2)) <= math.pow(rad,2) then
			
				if currpt <= 2 then
					if foundpt then
						if currpt <= 0 then
							break
						else
							tgt_bold[{currh,currw}]=0
						end	
					else
						foundpt = true
						currpt = math.floor(math.max(tgt:size(1),tgt:size(2))/2)+1
					end
				else
					if foundpt then
						tgt_bold[{currh,currw}]=0
					elseif tgt[{currh,currw}] > 0.05 * tgt:max() then
						foundpt = true
					end
				end
			else
				--tgt[{currh,currw}]=1
			end
		end
		
		currpt = currpt - 1
	end
	
end

function create_bold_image(tgt,numangles,hcnt,wcnt, ohcnt,owcnt,rad)

	local tgt_bold = torch.ones(tgt:size(1),tgt:size(2))
	
	local rettns = torch.zeros(numangles)
	local ang = 0
	local step = 2 * math.pi / numangles

	rettns:apply(function()
		ang = ang + step
		create_bold_image_ray(tgt, tgt_bold, ang, hcnt, wcnt, ohcnt,owcnt,rad)
		end)
	
	return tgt_bold
end

local function pixelate_image(img, window)
	local imgcpy = torch.zeros(math.floor((img:size(1)-1)/window)+1,math.floor((img:size(2)-1)/window)+1)
	local maxval = img:max()
	local i = 1
	local j = 0
	imgcpy:apply(function()
	
		if j == imgcpy:size(2) then
			i = i+1
			j = 1
		else
			j = j + 1
		end
		
		local hmin = math.max((i-1) * window + 1, 1)
		local wmin = math.max((j-1) * window + 1, 1)
		local hmax = math.min((i+1) * window, img:size(1))
		local wmax = math.min((j+1) * window, img:size(2))
		
		local ret = img:sub(hmin,hmax,wmin,wmax):max()
		
		if ret > 0.01 * maxval then
			return 1.0
		else
			return 0.0
		end
			

		end)
	return imgcpy/(imgcpy:max() + 0.0000000000000001)
end

function parse_candidates(fname,num)
	local source_centerpoints = torch.zeros(num,2)
	local target_centerpoints = torch.zeros(num,2)
	local inliers = torch.zeros(num)
	local file = io.open(fname,'r')
	local l,n,h,w
	for i = 1,num do
		l = file:read()
		n = tonumber(string.match(l,'%d+'))
		if not (n==i-1) then
			print('uh ohs when parsing')
			break
		end
		l = file:read()
		n = tonumber(string.match(l,'%d+'))
		inliers[i] = n
		l = file:read()
		for w,h in string.gmatch(l, '(%d+),%s(%d+)') do
			source_centerpoints[i] = torch.Tensor({tonumber(h),tonumber(w)})
		end
		l = file:read()
		--[[]]
		for w,h in string.gmatch(l, '(%d+),%s(%d+)') do
		--[[]]
			target_centerpoints[i] = torch.Tensor({tonumber(h),tonumber(w)})
		end
	end
	return inliers, source_centerpoints,target_centerpoints
end

function satisfy_hard_thresholds(test_val, params, quality)

	--[[
	PARAMS:
		gd_thresh_score = 0.6,
		gd_thresh_vrs_no_cap = 0.10,
		gd_thresh_vrs_cap = 30/360,
		gd_thresh_vid_rough = 0.40,
		gd_thresh_vid_fine = 0.80,

		mb_thresh_score = 0.75,
		mb_thresh_vrs_no_cap = 0.20,
		mb_thresh_vrs_cap = 90/360,
		mb_thresh_vid_rough = 0.80,
		mb_thresh_vid_fine = 0.975
	]]
	
	--[[
	ORDER:
		score
		vrs_no_cap
		vrs_capped
		vid_fine_tot
		vid_fine_tgt
		vid_fine_src
		vid_rough_tot
		vid_rough_tgt
		vid_rough_src
	]]
	
	local ret = false
	
	if not (quality and (quality ==  'gd' or quality == 'mb' or quality == 'am')) then
		quality = 'gd'
	end
	
	local thresh_score 		= params.mb_thresh_score
	local thresh_vrs_no_cap = params.mb_thresh_vrs_no_cap
	local thresh_vrs_cap 	= params.mb_thresh_vrs_cap
	local thresh_vid_rough 	= params.mb_thresh_vid_rough
	local thresh_vid_fine 	= params.mb_thresh_vid_fine
	
	if quality == 'gd' then
		thresh_score 		= params.gd_thresh_score
		thresh_vrs_no_cap 	= params.gd_thresh_vrs_no_cap
		thresh_vrs_cap 		= params.gd_thresh_vrs_cap
		thresh_vid_rough 	= params.gd_thresh_vid_rough
		thresh_vid_fine 	= params.gd_thresh_vid_fine
	elseif quality == 'am' then
		thresh_score 		= params.am_thresh_score
		thresh_vrs_no_cap 	= params.am_thresh_vrs_no_cap
		thresh_vrs_cap 		= params.am_thresh_vrs_cap
		thresh_vid_rough 	= params.am_thresh_vid_rough
		thresh_vid_fine 	= params.am_thresh_vid_fine
	end
	
	if test_val[1] <= thresh_score and
	   test_val[2] <= thresh_vrs_no_cap and
	   test_val[3] <= thresh_vrs_cap and
	   test_val[4] <= thresh_vid_fine and
	   test_val[7] <= thresh_vid_rough then
		ret = true
	end
	
	return ret
end

function satisfy_soft_thresholds(test_val, best_val, params)

	--[[
	PARAMS:
		perc_of_best_score = 1.25,
		perc_of_best_vrs = 2.00,
		perc_of_best_vid = 1.25
	]]
	
	--[[
	ORDER:
		score
		vrs_no_cap
		vrs_capped
		vid_fine_tot
		vid_fine_tgt
		vid_fine_src
		vid_rough_tot
		vid_rough_tgt
		vid_rough_src
	]]
	local ret = false
	
	local thresh_score 	= params.perc_of_best_score
	local thresh_vrs 	= params.perc_of_best_vrs
	local thresh_vid 	= params.perc_of_best_vid
	
	if test_val[1] <= thresh_score * best_val[1] and
	   test_val[2] <= thresh_vrs   * best_val[2] and
	   test_val[3] <= thresh_vrs   * best_val[3] and
	   test_val[4] <= thresh_vid   * best_val[4] and
	   test_val[7] <= thresh_vid   * best_val[7] then
		ret = true
	end

	return ret
end

function find_candidates(dirb, fnamestart,fnameend,params)
	
	local radius_to_scale 		= params.radius_max			/	params.scale
	local center_dist_to_scale 	= params.center_dist_max	/	params.scale
	local pixelate_to_scale 	= params.pixelate_max		/	params.scale
	local wiggle_to_scale 		= params.wiggle_max			/	params.scale
	local vrs_cap_to_scale 		= params.vrs_cap			/	params.scale
	
	local pix_wiggle			= wiggle_to_scale			/	pixelate_to_scale
	local pix_radius 			= radius_to_scale			/	pixelate_to_scale
	local pix_vrs_cap 			= vrs_cap_to_scale			/	pixelate_to_scale
	
	local numangles_vrs = params.numangles_vrs
	local numangles_bold = params.numangles_bold
	
	local best_scores_for_all = torch.Tensor(fnameend-fnamestart+1,10)
	local z = 0
	
	for i=fnamestart,fnameend do
		z = z + 1

		collectgarbage()
		
		local grt_candidates = {}
		local good_candidates = {}
		local maybe_candidates = {}
	
		local j = i+1
	
		local dir = dirb..'/'..i..'_'..j
		local dir_out = dirb..'/output/'
		local centersfile = dir..'/candidates.txt'
		local files = util.fs.files_only(dir)
		local n = #files-1
		if files[1] == dir..'/.DS_Store' then
			n = n-1
		end
	
		local numinliers,src_cntpts,tgt_cntpts = parse_candidates(centersfile,n)
		
		--[[
		local mean_inliers = numinliers:mean()
		local max_inliers = numinliers:max()
		local test_inliers_threshold = math.min(mean_inliers,max_inlers/2)
		]]
		
		local candidate_scores = torch.zeros(n,9) + math.huge
		
		--[[
			ORDER:
			score
			vrs_no_cap
			vrs_capped
			vid_fine_tot
			vid_fine_tgt
			vid_fine_src
			vid_rough_tot
			vid_rough_tgt
			vid_rough_src
		]]
		
		print()
		print('COMPUTING CANDIDATES FOR '..i..'_'..j)
		print()
		
		local thresh_score 		= params.mb_thresh_score
		local thresh_vrs_no_cap = params.mb_thresh_vrs_no_cap
		local thresh_vrs_cap 	= params.mb_thresh_vrs_cap
		local thresh_vid_rough 	= params.mb_thresh_vid_rough
		local thresh_vid_fine 	= params.mb_thresh_vid_fine
	
		for k=0,n-1 do
	
			collectgarbage()
	
			local inliers = numinliers[k+1]
			
			if inliers >= math.max(numinliers:mean(),numinliers:max()/2) then
		
			local tcnt = tgt_cntpts[k+1]
			local scnt = src_cntpts[k+1]
			
			local score = math.huge
			local vrs_no_cap = math.huge
			local vrs_capped = math.huge
			local vid_fine_tot = math.huge
			local vid_fine_tgt = math.huge
			local vid_fine_src = math.huge
			local vid_rough_tot = math.huge
			local vid_rough_tgt = math.huge
			local vid_rough_src = math.huge
			
			local scores= torch.Tensor({score, vrs_no_cap, vrs_capped, 
										vid_fine_tot, vid_fine_tgt, vid_fine_src,
										vid_rough_tot, vid_rough_tgt,vid_rough_src})
		
			local imname = dir..'/'..k..'.png'
			local imntgt = dir_out..i..'_'..j..'_'..k..'_tgt.png'
			local imnsrc = dir_out..i..'_'..j..'_'..k..'_src.png'
			local pxntgt = dir_out..i..'_'..j..'_'..k..'_pix_tgt.png'
			local pxnsrc = dir_out..i..'_'..j..'_'..k..'_pix_src.png'
				
			local imgcls = wand.new(imname)	
			local imgtns = imgcls:toTensor('double','RGB','DHW')
			local hght = imgtns:size(2)
			local wdth = imgtns:size(3)
		
			local imgtgt = imgtns[2]:clone()
			local imgsrc = imgtns[1]:clone()
				
			local img_thcnt = tgt_cntpts[k+1][1]
			local img_twcnt = tgt_cntpts[k+1][2]
			local img_shcnt = src_cntpts[k+1][1]
			local img_swcnt = src_cntpts[k+1][2]
				
			local img_min_h = math.min(img_thcnt,img_shcnt)-radius_to_scale
			local img_max_h = math.max(img_thcnt,img_shcnt)+radius_to_scale
			local img_min_w = math.min(img_twcnt,img_swcnt)-radius_to_scale
			local img_max_w = math.max(img_twcnt,img_swcnt)+radius_to_scale
			
			local pixtgt = pixelate_image(imgtgt,pixelate_to_scale)
			local pixsrc = pixelate_image(imgsrc,pixelate_to_scale)
			local pix_hght = pixtgt:size(1)
			local pix_wdtht = pixtgt:size(2)
				
			vid_fine_tot, vid_fine_tgt, vid_fine_src = validation_imgdiff(imgtgt,imgsrc,img_min_h, img_max_h, img_min_w, img_max_w)
			
			scores= torch.Tensor({score, vrs_no_cap, vrs_capped, 
										vid_fine_tot, vid_fine_tgt, vid_fine_src,
										vid_rough_tot, vid_rough_tgt,vid_rough_src})
										
			if vid_fine_tot <= thresh_vid_fine then
				
				local pix_thcnt = math.floor(tgt_cntpts[k+1][1]/pixelate_to_scale) + 1
				local pix_twcnt = math.floor(tgt_cntpts[k+1][2]/pixelate_to_scale) + 1
				local pix_shcnt = math.floor(src_cntpts[k+1][1]/pixelate_to_scale) + 1
				local pix_swcnt = math.floor(src_cntpts[k+1][2]/pixelate_to_scale) + 1
				
				local pix_min_h = math.min(pix_thcnt,pix_shcnt)-pix_radius
				local pix_max_h = math.max(pix_thcnt,pix_shcnt)+pix_radius
				local pix_min_w = math.min(pix_twcnt,pix_swcnt)-pix_radius
				local pix_max_w = math.max(pix_twcnt,pix_swcnt)+pix_radius
						
				vid_rough_tot, vid_rough_tgt, vid_rough_src = validation_imgdiff(pixtgt,pixsrc,pix_min_h, pix_max_h, pix_min_w, pix_max_w)
				
				scores= torch.Tensor({score, vrs_no_cap, vrs_capped, 
										vid_fine_tot, vid_fine_tgt, vid_fine_src,
										vid_rough_tot, vid_rough_tgt,vid_rough_src})
				
				if vid_rough_tot <= thresh_vid_rough then
				
					local pixtgt_bold = create_bold_image(pixtgt, numangles_bold, pix_thcnt, pix_twcnt, pix_shcnt, pix_swcnt, pix_radius)
					local pixsrc_bold = create_bold_image(pixsrc, numangles_bold, pix_shcnt, pix_swcnt, pix_thcnt, pix_twcnt, pix_radius)
				
					--[[
					image.save(pxntgt,pixtgt_bold)
					image.save(pxnsrc,pixsrc_bold)
					--[[]]
				
					local vrs1_no_cap = validation_ray(pixtgt,      pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle)
					local vrs2_no_cap = validation_ray(pixsrc,      pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle)
					local vrs3_no_cap = validation_ray(pixtgt_bold, pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle)
					local vrs4_no_cap = validation_ray(pixsrc_bold, pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle)
					vrs_no_cap = (math.max(vrs1_no_cap,vrs2_no_cap)+math.max(vrs3_no_cap,vrs4_no_cap))/2
					
					scores= torch.Tensor({score, vrs_no_cap, vrs_capped, 
										vid_fine_tot, vid_fine_tgt, vid_fine_src,
										vid_rough_tot, vid_rough_tgt,vid_rough_src})
					
					if vrs_no_cap <= thresh_vrs_no_cap then
				
						local vrs1_capped = validation_ray(pixtgt,      pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle, pix_vrs_cap)
						local vrs2_capped = validation_ray(pixsrc,      pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle, pix_vrs_cap)
						local vrs3_capped = validation_ray(pixtgt_bold, pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle, pix_vrs_cap)
						local vrs4_capped = validation_ray(pixsrc_bold, pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle, pix_vrs_cap)
						vrs_capped = (math.max(vrs1_capped,vrs2_capped)+math.max(vrs3_capped,vrs4_capped))/2
						
						score = (vid_fine_tot + vid_rough_tot + vrs_no_cap + vrs_capped)/4
						
						scores= torch.Tensor({score, vrs_no_cap, vrs_capped, 
										vid_fine_tot, vid_fine_tgt, vid_fine_src,
										vid_rough_tot, vid_rough_tgt,vid_rough_src})
				
						--[[if not (vrs_capped <= thresh_vrs_cap) then
							print (scores)
															  
						end -- if vrs_capped satisfies thresh	]]							  
						
					end -- if vrs_no_cap satisfies thresh								  
					
				end -- if vid_rough satisfies thresh								  
				
			end	-- if vid_fine satisfies thresh								  
												  
			candidate_scores[k+1] = scores	
		
		end
		end
		collectgarbage()
		
		local min_score 		= candidate_scores:select(2,1):min()
		local min_vrs_no_cap 	= candidate_scores:select(2,2):min()
		local min_vrs_capped 	= candidate_scores:select(2,3):min()
		local min_vid_fine_tot 	= candidate_scores:select(2,4):min()
		local min_vid_fine_tgt 	= candidate_scores:select(2,5):min()
		local min_vid_fine_src 	= candidate_scores:select(2,6):min()
		local min_vid_rough_tot = candidate_scores:select(2,7):min()
		local min_vid_rough_tgt = candidate_scores:select(2,8):min()
		local min_vid_rough_src = candidate_scores:select(2,9):min()
		
		local best_scores = torch.Tensor({min_score,
										  min_vrs_no_cap,
										  min_vrs_capped,
										  min_vid_fine_tot,
										  min_vid_fine_tgt,
										  min_vid_fine_src,
										  min_vid_rough_tot,
										  min_vid_rough_tgt,
										  min_vid_rough_src, 
										  z})
										  
		best_scores_for_all[z] = best_scores
		
		local found_am = false
		local found_gd = false
		
		if satisfy_hard_thresholds(best_scores, params, 'mb') then
		
			for k=0,n-1 do
		
				collectgarbage()
			
				-- satisfy hard thresholds
				local scores = candidate_scores[k+1]
			
				if satisfy_soft_thresholds(scores, best_scores, params) then
					if found_am and satisfy_hard_thresholds(scores, params, 'am') then
						grt_candidates[#grt_candidates+1] = {k, scores}
					elseif found_gd and satisfy_hard_thresholds(scores, params, 'gd') then
						 if satisfy_hard_thresholds(scores, params, 'am') then
							grt_candidates[#grt_candidates+1] = {k, scores}
							found_am = true
						else
							good_candidates[#good_candidates+1] = {k, scores}
						end
					elseif satisfy_hard_thresholds(scores, params, 'am') then
						grt_candidates[#grt_candidates+1] = {k, scores}
						found_am = true
					elseif satisfy_hard_thresholds(scores, params, 'gd') then
						good_candidates[#good_candidates+1] = {k, scores}
						found_gd = true
					elseif satisfy_hard_thresholds(scores, params, 'mb') then
						maybe_candidates[#maybe_candidates+1] = {k, scores}
					end
				end
			end
		end
		
		local candidate_set = {}
		local candidate_flag = 0
		if #grt_candidates > 0 then
			candidate_set = grt_candidates
			candodate_flag = 3
		elseif #good_candidates > 0 then
			candidate_set = good_candidates
			candidate_flag = 2
		elseif #maybe_candidates > 0 then
			candidate_set = maybe_candidates
			candidate_flag = 1
		end
		
		table.sort(candidate_set, function(a,b)
								      if  a[2][1] < b[2][1] then
								      	return true
								      end
								      if a[2][1] == b[2][1] then
								      	if a[2][7] < b[2][7] then
								      		return true
								      	end
								      	if a[2][7] == b[2][7] then
								      		if a[2][4] < b[2][4] then
								      			return true
								      		end
								      		if a[2][4] == b[2][4] and a[2][2] + a[2][3] < b[2][2] + b[2][3] then
								      			return true
								      		end
								      	end
								      end
								      return false
								  end)
		
		-- top 5 candidates
		for t=1,math.min(#candidate_set,3) do
			local k = candidate_set[t][1]
			local scores = candidate_set[t][2]
			
			if candidate_flag > 0 then
				local imname = dir..'/'..k..'.png'
				local imgcls = wand.new(imname)
				local outname = dir_out..'candidate_'..t..'_'..i..'_'..j..'_'..k..'.png'
				local prev_spaces = ''
				local descriptor = 'FOUND '
				if candidate_flag == 1 then --maybe
					prev_spaces = '                                    '
					descriptor = prev_spaces..'MAYBE '
					outname = dir_out..'candidate_'..t..'_'..i..'_'..j..'_'..k..'_mb.png'
				elseif candidate_flag == 2 then --good
					prev_spaces = '                  '
					descriptor = prev_spaces..'GOOD '
					outname = dir_out..'candidate_'..t..'_'..i..'_'..j..'_'..k..'_gd.png'
				elseif candidate_flag == 3 then --amazing
					prev_spaces = '   '
					descriptor = prev_spaces..descriptor..'AMAZING '
					outname = dir_out..'candidate/candidate_'..t..'_'..i..'_'..j..'_'..k..'_am.png'
				else
					outname = dir_out..'reject/candidate_'..t..'_'..i..'_'..j..'_'..k..'_rj.png'
					prev_spaces = '                                                   '
					descriptor = prev_spaces..'REJECT '
				end
	
				print()				
				print( descriptor..'CANDIDATE FOR '..i..'_'..j..' : '..k)
				print(prev_spaces..'           SCORE: '..scores[1]..' ; BEST: '..best_scores[1])
				print(prev_spaces..'      VRS_no_cap: '..scores[2]..' ; BEST: '..best_scores[2])
				print(prev_spaces..'      VRS_capped: '..scores[3]..' ; BEST: '..best_scores[3])
				print(prev_spaces..'    VID_fine_TOT: '..scores[4]..' ; BEST: '..best_scores[4])
				print(prev_spaces..'    vid_fine_tgt: '..scores[5]..' ; BEST: '..best_scores[5])
				print(prev_spaces..'    vid_fine_src: '..scores[6]..' ; BEST: '..best_scores[6])
				print(prev_spaces..'   VID_rough_TOT: '..scores[7]..' ; BEST: '..best_scores[7])
				print(prev_spaces..'   vid_rough_tgt: '..scores[8]..' ; BEST: '..best_scores[8])
				print(prev_spaces..'   vid_rough_src: '..scores[9]..' ; BEST: '..best_scores[9])
				
				imgcls:save(outname)
			end
		end
		collectgarbage()
		
	end
	collectgarbage()
	
	return best_scores_for_all
end

bs = find_candidates(dirb,fnamestart,fnameend,parameters)

print()
print(bs)

collectgarbage()