local hough = image.hough
local wand = image.Wand
local io = require 'io'
local path = require 'path'
local fs = require 'fs'

local validation = Class()

validation.default_parameters = {
	scale = 0.015,
	radius_max = 5.0,
	center_dist_max = 5.0,
	center_dist_min = 0.1,

	pixelate_max = 0.30,
	wiggle_max = 0.25,
	vrs_cap = 0.5,
	numangles_vrs = 360,
	numangles_bold = 720,
	
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
	--print('totalPixels: '..tns1cpy:nElement()..'; numberOfColoredPixels: '..numSum..'; overlapPixels: '..ovlp)
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

local function create_bold_image_ray(tgt, tgt_bold, angle, hcnt, wcnt, ohcnt, owcnt,rad)

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

local function create_bold_image(tgt,numangles,hcnt,wcnt, ohcnt,owcnt,rad)

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

function validation.compute_score (combined_img, src_cnt_h, src_cnt_w, tgt_cnt_h, tgt_cnt_w, bname, params)

	if not params then
		params = validation.default_parameters
	end
	
	local radius_to_scale 		= params.radius_max			/	params.scale
	local cntr_dist_mx_to_scale = params.center_dist_max	/	params.scale
	local cntr_dist_mn_to_scale = params.center_dist_min	/	params.scale
	local pixelate_to_scale 	= params.pixelate_max		/	params.scale
	local wiggle_to_scale 		= params.wiggle_max			/	params.scale
	local vrs_cap_to_scale 		= params.vrs_cap			/	params.scale
	
	local pix_wiggle			= wiggle_to_scale			/	pixelate_to_scale
	local pix_radius 			= radius_to_scale			/	pixelate_to_scale
	local pix_vrs_cap 			= vrs_cap_to_scale			/	pixelate_to_scale
	
	local numangles_vrs = params.numangles_vrs
	local numangles_bold = params.numangles_bold
	
	if (torch.Tensor({src_cnt_h,src_cnt_w}):dist(torch.Tensor({tgt_cnt_h,tgt_cnt_w})) > cntr_dist_mx_to_scale) or
	   (torch.Tensor({src_cnt_h,src_cnt_w}):dist(torch.Tensor({tgt_cnt_h,tgt_cnt_w})) < cntr_dist_mn_to_scale) then
		return torch.ones(9)
	end
	
	local hght = combined_img:size(2)
	local wdth = combined_img:size(3)
	
	local imgtgt = combined_img[2]:clone()
	local imgsrc = combined_img[1]:clone()
	
	--[[image.save('/Users/lihui815/tmp/'..bname..'_'..'img_tgt.png',imgtgt)
	image.save('/Users/lihui815/tmp/'..bname..'_'..'img_src.png',imgsrc)]]
	
	local img_min_h = math.min(tgt_cnt_h,src_cnt_h)-radius_to_scale
	local img_max_h = math.max(tgt_cnt_h,src_cnt_h)+radius_to_scale
	local img_min_w = math.min(tgt_cnt_w,src_cnt_w)-radius_to_scale
	local img_max_w = math.max(tgt_cnt_w,src_cnt_w)+radius_to_scale
				
	local score = math.huge
	local vrs_no_cap = math.huge
	local vrs_capped = math.huge
	local vid_fine_tot = math.huge
	local vid_fine_tgt = math.huge
	local vid_fine_src = math.huge
	local vid_rough_tot = math.huge
	local vid_rough_tgt = math.huge
	local vid_rough_src = math.huge
					
	local pixtgt = pixelate_image(imgtgt,pixelate_to_scale)
	local pixsrc = pixelate_image(imgsrc,pixelate_to_scale)
	local pix_hght = pixtgt:size(1)
	local pix_wdtht = pixtgt:size(2)
	
	--[[image.save('/Users/lihui815/tmp/'..bname..'_'..'pixelated_tgt.png',pixtgt)
	image.save('/Users/lihui815/tmp/'..bname..'_'..'pixelated_src.png',pixsrc)]]
				
	local vid_fine_tot, vid_fine_tgt, vid_fine_src = validation_imgdiff(imgtgt,imgsrc,img_min_h, img_max_h, img_min_w, img_max_w)
				
	local pix_thcnt = math.floor((tgt_cnt_h-1)/pixelate_to_scale) + 1
	local pix_twcnt = math.floor((tgt_cnt_w-1)/pixelate_to_scale) + 1
	local pix_shcnt = math.floor((src_cnt_h-1)/pixelate_to_scale) + 1
	local pix_swcnt = math.floor((src_cnt_w-1)/pixelate_to_scale) + 1
				
	local pix_min_h = math.min(pix_thcnt,pix_shcnt)-pix_radius
	local pix_max_h = math.max(pix_thcnt,pix_shcnt)+pix_radius
	local pix_min_w = math.min(pix_twcnt,pix_swcnt)-pix_radius
	local pix_max_w = math.max(pix_twcnt,pix_swcnt)+pix_radius
						
	local vid_rough_tot, vid_rough_tgt, vid_rough_src = validation_imgdiff(pixtgt,pixsrc,pix_min_h, pix_max_h, pix_min_w, pix_max_w)
					
	local pixtgt_bold = create_bold_image(pixtgt, numangles_bold, pix_thcnt, pix_twcnt, pix_shcnt, pix_swcnt, pix_radius)
	local pixsrc_bold = create_bold_image(pixsrc, numangles_bold, pix_shcnt, pix_swcnt, pix_thcnt, pix_twcnt, pix_radius)
	
	--[[image.save('/Users/lihui815/tmp/'..bname..'_'..'pixbold_tgt.png',pixtgt_bold)
	image.save('/Users/lihui815/tmp/'..bname..'_'..'pixbold_src.png',pixsrc_bold)]]
							
	local vrs1_no_cap = validation_ray(pixtgt,      pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle)
	local vrs2_no_cap = validation_ray(pixsrc,      pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle)
	local vrs3_no_cap = validation_ray(pixtgt_bold, pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle)
	local vrs4_no_cap = validation_ray(pixsrc_bold, pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle)
	local vrs_no_cap = (math.max(vrs1_no_cap,vrs2_no_cap)+math.max(vrs3_no_cap,vrs4_no_cap))/2
					
	local vrs1_capped = validation_ray(pixtgt,      pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle, pix_vrs_cap)
	local vrs2_capped = validation_ray(pixsrc,      pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle, pix_vrs_cap)
	local vrs3_capped = validation_ray(pixtgt_bold, pixsrc_bold, numangles_vrs, pix_thcnt, pix_twcnt, pix_wiggle, pix_vrs_cap)
	local vrs4_capped = validation_ray(pixsrc_bold, pixtgt_bold, numangles_vrs, pix_shcnt, pix_swcnt, pix_wiggle, pix_vrs_cap)
	local vrs_capped = (math.max(vrs1_capped,vrs2_capped)+math.max(vrs3_capped,vrs4_capped))/2
						
	local score = (vid_fine_tot + vid_rough_tot + vrs_no_cap + vrs_capped)/4
						
	local all_scores= torch.Tensor({score, 			vrs_no_cap, 	vrs_capped, 
									vid_fine_tot, 	vid_fine_tgt, 	vid_fine_src,
									vid_rough_tot, 	vid_rough_tgt,	vid_rough_src})
				
	collectgarbage()
	return all_scores
end

collectgarbage()