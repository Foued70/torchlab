path = require 'path'
saliency = require '../image/saliency'
io = require 'io'
Pointcloud = PointCloud.Pointcloud

nms = image.NonMaximalSuppression.new(5,5)

function register_image_to_pc(img, xyz_map, rmask, emask, phi, theta, nmpi,
															ih,iw,height,width,rad_per_pix,vscale,hhov,vhov,hcen,thof,hhof)

	lmsk = theta:le(0):cmul(emask)
	gmsk = theta:gt(0):cmul(emask)
	theta[lmsk] = theta[lmsk]*(-1)+math.pi
	theta[gmsk] = theta[gmsk]*(-1)+math.pi
	theta[emask] = theta[emask]-thof
	lmsk = theta:le(0):cmul(emask)
	gmsk = theta:gt(2*math.pi):cmul(emask)
	theta[lmsk] = theta[lmsk] + 2*math.pi
	theta[gmsk] = theta[gmsk] - 2*math.pi

	pix_w = theta:clone()
	pix_w[emask] = pix_w[emask]/rad_per_pix
	pix_w[emask] = pix_w[emask]:clone():apply(function(w) return (w%iw)+1 end)
	pix_w[rmask] = 0

	xyz = xyz_map:clone()
	xyz[3] = -xyz[3]+hhof
	xyz[3][rmask] = 0
	depth = xyz:norm(2,1):squeeze()
	phi = xyz[3]:clone():cdiv(depth):asin()
	phi[rmask]= 0

	pix_h = phi/(vscale*rad_per_pix)+(hcen+1)
	pix_h[rmask]=0

	rrr = torch.zeros(height,width)
	ggg = torch.zeros(height,width)
	bbb = torch.zeros(height,width)
	msk = torch.zeros(height,width)

	im_r = img[1]
	im_g = img[2]
	im_b = img[3]

	black = img:norm(2,1):squeeze():eq(0):clone()

	pix_h = pix_h:apply(function(h)
												f = math.floor(h)
												c = math.ceil(h)
												if math.abs(h-f) <= math.abs(c) then
													return f
												else
													return c
												end
											end)
									
	pix_w = pix_w:apply(function(w)
												f = math.floor(w)
												c = math.ceil(w)
												if math.abs(w-f) <= math.abs(c) then
													return f
												else
													return c
												end
											end)

	--[[]]
	for h = 1,height do
		imhh = pix_h[h]:clone()
		imwh = pix_w[h]:clone()
		emkh = emask[h]:clone()
		if emkh:double():sum() > 0 then
			for w = 1,width do
				if emkh[w] > 0 then
					imh = imhh[w]
					imw = imwh[w]
					if imh > 0 and imh <= ih and imw > 0 and imw <= iw and black[imh][imw] == 0 then
						rrr[h][w] = im_r[imh][imw]
						ggg[h][w] = im_g[imh][imw]
						bbb[h][w] = im_b[imh][imw]
						msk[h][w] = 1
					end
				end
			end
		end
	end

	rgb_map = torch.zeros(3,height,width)
	rgb_map[1] = rrr
	rgb_map[2] = ggg
	rgb_map[3] = bbb
	msk[rmask] = 0
	num = msk:double():sum()

	sal_nm = saliency.high_entropy_features(nmpi)
	sal_cl = saliency.high_entropy_features(rgb_map:mean(1):squeeze())
	
	sal_nm = sal_nm:div(sal_nm:max()+0.0001)
	sal_cl = sal_cl:div(sal_cl:max()+0.0001)
	
	sal_nm[rmask] = 0
	sal_cl[rmask] = 0
	
	sal_nm_sp = nms:forward(sal_nm:clone())
	sal_cl_sp = nms:forward(sal_cl:clone())
	
	sal_nm = sal_nm:clone()*1 + sal_nm_sp:clone():sqrt()*4 + sal_nm:clone():cmul(sal_nm_sp)*4
	sal_cl = sal_cl:clone()*2 + sal_cl_sp:clone():sqrt()*4 + sal_cl:clone():cmul(sal_cl_sp)*3
	
	sal_nm = sal_nm:div(sal_nm:max()+0.0001)
	sal_cl = sal_cl:div(sal_cl:max()+0.0001)
	
	sal_nm[rmask] = 0
	sal_cl[rmask] = 0
	
	score = sal_nm:dist(sal_cl)
	
	sal_im = sal_nm:repeatTensor(3,1,1):clone()
	sal_im[2] = sal_cl
	sal_im:select(1,3):fill(0)

	collectgarbage()

	return rgb_map, sal_im, score
end

for sw = 5,22 do

  if not(sw == 11) then
  
	topdir = '/Users/lihui815/Documents'
	prjdir = 'elegant-prize-3149' --'precise-transit-6548'--'mobile-void-0590'--'elegant-prize-3149'

	srcdir = 'source/po_scan/a/'
	wrkdir = 'work/a_00'
	imgdir = 'Aligned'
	sweepn = ''..sw
	while #sweepn < 3 do
		sweepn = '0'..sweepn
	end

	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'NMP'))
	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'RGB'))
	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'XYZRGB'))
	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'OD'))
	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'TXT'))
	util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'SAL'))

	pcfile = path.join(topdir,prjdir,srcdir,sweepn,'sweep.xyz')
	imfile = path.join(topdir,prjdir,wrkdir,imgdir,sweepn,'enblend_panorama_360_lambda_0.125_post.png')
	nmfile = path.join(topdir,prjdir,wrkdir,'NMP/nmp'..sweepn..'.png')
	clfile = path.join(topdir,prjdir,wrkdir,'RGB/rgb'..sweepn..'.png')
	xyfile = path.join(topdir,prjdir,wrkdir,'XYZRGB/xyz'..sweepn..'.xyz')
	odfile = path.join(topdir,prjdir,wrkdir,'OD/pc'..sweepn..'.od')
	otfile = path.join(topdir,prjdir,wrkdir,'TXT/'..sweepn..'.txt')
	slfile = path.join(topdir,prjdir,wrkdir,'SAL/'..sweepn..'.png')

	dbgf = io.open(otfile,'w')


	pc = PointCloud.PointCloud.new(pcfile)
	img = image.load(imfile)
	height = pc.height
	width = pc.width
	ih = img:size(2)
	iw = img:size(3)
	rad_per_pix = 2*math.pi/iw
	vscale = 1.15
	hhov = 2*math.pi
	vhov = ih*rad_per_pix*vscale
	xyz_map = pc:get_xyz_map_no_mask()
	index,rmask = pc:get_index_and_mask()
	emask = rmask:eq(0)
	phi,theta = pc:get_xyz_phi_theta()
	nmp,_G.ndd = pc:get_normal_map()
	nmpi = image.combine(nmp)

	--[[]]
	--elegant-prize
	hcen_init = (1/2)*ih+(5/128)*ih --(- goes down, + goes up)
	thof_init = 43*math.pi/100 --(- goes left, + goes right)
	hhof_init = -104 --(- goes down, + goes up)
	--[[]]

	--[[
	--mobile-void
	hcen_init = (1/2)*ih+(5/128)*ih --(- goes down, + goes up)
	thof_init = 30*math.pi/100 --(- goes left, + goes right)
	hhof_init = -104 --(- goes down, + goes up)
	--[[]]

	--[[
	--precise-transit
	hcen_init = (1/2)*ih+(5/128)*ih
	thof_init = pc.meta_properties.camera_offset_azimuth-(7/720)*math.pi
	hhof_init = pc.meta_properties.camera_offset_z
	--[[]]

	hcen_wiggle = 2
	thof_wiggle = 2*math.pi/720
	hhof_wiggle = 2.5
	numwiggle = 2

  dbg_str = ''..(-1)..', '..(-1)..': best score found at ['..'0'..', '..hcen_init..', '..thof_init..', '..hhof_init..']: '..math.huge..'\n'
	dbgf:write(dbg_str)
	print(dbg_str)


	best_score = math.huge
	best_rgb = torch.zeros(3,ih,iw)
	best_sal = torch.zeros(3,ih,iw)
	best_hcen = hcen_init
	best_thof = thof_init
	best_hhof = hhof_init
																					 
	iter = 0                                                     
	while iter < 5*3 do
	
		hcen = best_hcen
		thof = best_thof
		hhof = best_hhof
		bwig = 0
	
		local typ = iter % 3
	
		for wiggle = -numwiggle,numwiggle do
		
			if typ == 0 then
				hcen = hcen_init + wiggle*hcen_wiggle 
			elseif typ == 1 then
				thof = thof_init + wiggle*thof_wiggle
			else
				hhof = hhof_init + wiggle*hhof_wiggle
			end
		
			local rgb, sal, score =  register_image_to_pc(img:clone(), xyz_map:clone(), rmask:clone(), 
																												 emask:clone(), phi:clone(), theta:clone(), nmpi:clone(),
																												 ih,iw,height,width,rad_per_pix,vscale,hhov,vhov,hcen,thof,hhof)
																										
      dbg_str = '     ['..wiggle..', '..hcen..', '..thof..', '..hhof..']: '..score..'\n'
			dbgf:write(dbg_str)
	    print(dbg_str)
			
			if score < best_score then
				best_score = score
				best_rgb = rgb
				best_sal = sal
				best_hcen = hcen
				best_thof = thof
				best_hhof = hhof
				bwig = wiggle
	
				dbg_str = '        found new best\n'
				dbgf:write(dbg_str)
  	    print(dbg_str)
			end
		
		end
	
		hcen_init = best_hcen
		thof_init = best_thof
		hhof_init = best_hhof
	
		if typ == 2 then
			hcen_wiggle = hcen_wiggle/(numwiggle+1)
			thof_wiggle = thof_wiggle/(numwiggle+1)
			hhof_wiggle = hhof_wiggle/(numwiggle+1)
		end
		
		dbg_str = ''..math.floor(iter/3)..', '..typ..': best score found at ['..bwig..', '..best_hcen..', '..best_thof..', '..best_hhof..']: '..best_score..'\n'
  	dbgf:write(dbg_str)
	  print(dbg_str)
	
		iter = iter + 1
		
	end
	
	image.save(nmfile,nmpi)
	image.save(clfile,best_rgb)
	image.save(slfile,best_sal)

	pc:load_rgb_map(clfile)

	points_t = torch.zeros(3,num)
	rgb_t = torch.zeros(3,num)

	for i = 1,3 do
		points_t[i] = xyz_map[i][msk:byte()]
		rgb_t[i] = rgb_map[i][msk:byte()]
	end
	rgb_t:mul(255):floor()

	points = points_t:t()
	rgb = rgb_t:t()
	pc.save_any_points_to_xyz(xyfile,points,rgb)
	pc:save_to_od(odfile)

	--[[]]
	
	dbgf:close()

  end
	collectgarbage()
end
