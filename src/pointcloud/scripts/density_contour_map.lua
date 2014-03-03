cmd = torch.CmdLine()

cmd:text()
cmd:text('convert to pointcloud dat file')
cmd:text()
cmd:text('Options')
cmd:option(           '-srcdir', '/Users/lihui815/Documents/precise-transit-6548/work/a/pointcloud', '')
cmd:option(           '-savdir', '/Users/lihui815/Documents/precise-transit-6548/work/a/contour', '')
cmd:option(       '-resolution', 100, '')
cmd:option('-sampling_fraction', 0.001, '')
cmd:option(   '-scanner_radius', 500, '')

cmd:option('-interactive', false, '')
cmd:text()

params     = cmd:parse(process.argv)

function get_2d_density(pc, resolution, frac, dmin)

  dmin = dmin or 250
  local theta_tol    = 2 * math.pi/(pc.width * math.sqrt(frac))
  
  local vmask   = pc:get_valid_masks()
  local H       = pc:get_transformation_matrix()
  local flat = pc:get_flattened_image(resolution,nil,H)
  local img_h   = flat:size(2)
  local img_w   = flat:size(3)
  local img  = torch.zeros(img_h, img_w)

  local xyz, phi, theta = pc:get_xyz_map_unclipped(H)
  local nmp, nphi       = pc:get_normal_map()
  local depth = pc:get_depthxy_map_unclipped()
  local nmask = torch.rand(pc.height, pc.width):le(frac):cmul(depth:gt(dmin))
  local num   = nmask:double():sum()
  print(num)

	local th_msked = theta[nmask]
	local dp_msked = depth[nmask]

	local crdy   = torch.range(1,img_w):repeatTensor(img_h,1) - img_w/2
	local crdx   = torch.range(1,img_h):repeatTensor(img_w,1):t() - img_h/2

	local dist2d = (crdx:clone():pow(2) + crdy:clone():pow(2)):sqrt()
	local sinth  = crdy:clone():cdiv(dist2d)
	local costh  = crdx:clone():cdiv(dist2d)
	local tht2d  = costh:clone():acos()
	tht2d[sinth:lt(0)] = -tht2d[sinth:lt(0)]
	local incr   = torch.ones(dist2d:size()) - image.combine(dist2d)--:pow(2)

	for n = 1,num do
	
		local thn = th_msked[n]
		local dpn = dp_msked[n]
		local thm = (tht2d-thn):abs():lt(theta_tol)
		local dpm = (dist2d:le(dpn/resolution))
		local msk = thm:clone():cmul(dpm)
	
		img[msk] = img[msk]+incr[msk]

    if n % 1000 == 0 then
			print (num-n)
		end
		
		thn = nil
		dpn = nil
		thm = nil
		dpm = nil
		msk = nil
		collectgarbage()
	end

	img  = img:div(img:max()+0.000001)

	local n    = 5
	local ker  = image.combine(incr:sub(img_h/2-n,img_h/2+n, img_w/2-n,img_w/2+n):clone())
	local conv = torch.conv2(img,ker,'F'):sub(1+n,img_h+n,1+n,img_w+n):clone()
	img  = image.combine(conv):cmul(dist2d:gt(dmin/resolution):double())

	incr  = nil
	msk   = nil
	cur   = nil
	nex   = nil
	nmask = nil
	crdh  = nil
	crdw  = nil
	sinth = nil
	costh = nil
	th_msked = nil
	dp_msked = nil
	tht2d    = nil
	dist2d   = nil
	collectgarbage()
	--[[]]

	depth = nil
	xyz   = nil
	phi   = nil
	theta = nil
	nmp   = nil
	nphi  = nil
	vmask = nil
	collectgarbage()
	
	local cx   = pc:get_center_transformed(H):div(resolution)[1]
	local cy   = pc:get_center_transformed(H):div(resolution)[2]
	
	local sum1 = img:sum(1):gt(0):double():squeeze()
	local sum2 = img:sum(2):gt(0):double():squeeze()
	local msk1 = sum1:gt(0)
	local msk2 = sum2:gt(0)
	local ind1 = torch.range(1,img_w)[msk1]:clone()
	local ind2 = torch.range(1,img_h)[msk2]:clone()
	
	local minh = ind2:min()
	local maxh = ind2:max()
	local minw = ind1:min()
	local maxw = ind1:max()
	
	sum1 = nil
	sum2 = nil
	msk1 = nil
	msk2 = nil
	ind1 = nil
	ind2 = nil
	
	local prevx = (img_h+1)/2
	local prevy = (img_w+1)/2
	local indcx = (maxh+minh)/2
	local indcy = (maxw+minw)/2
	cx = cx + (indcx - prevx)
	cy = cy + (indcy - prevy)
	
	img  = img:sub(minh,maxh,minw,maxw):clone():contiguous()
	flat = flat:sub(1,3,minh,maxh,minw,maxw):clone():contiguous()
	
	return img, flat, cx, cy
end

function get_combined_density(img1, flat1, cx1, cy1, img2, flat2, cx2, cy2)
  
  local imgh_1 = img1:size(1)
  local imgw_1 = img1:size(2)
  
  local imgh_2 = img2:size(1)
  local imgw_2 = img2:size(2)
  
  local minx = math.min(cx1-imgh_1/2,cx2-imgh_2/2)
	local maxx = math.max(cx1+imgh_1/2,cx2+imgh_2/2)
	local miny = math.min(cy1-imgw_1/2,cy2-imgw_2/2)
	local maxy = math.max(cy1+imgw_1/2,cy2+imgw_2/2)

	local cx = (maxx + minx)/2
	local cy = (maxy + miny)/2

	local img_h = math.ceil(maxx-minx)
	local img_w = math.ceil(maxy-miny)

	local txb_1 = math.ceil(cx1 - imgh_1/2 - minx + 1)
	local txe_1 = math.ceil(txb_1 + imgh_1 -1)
	local tyb_1 = math.ceil(cy1 - imgw_1/2 - miny + 1)
	local tye_1 = math.ceil(tyb_1 + imgw_1 -1)

	local txb_2 = math.ceil(cx2 - imgh_2/2 - minx + 1)
	local txe_2 = math.ceil(txb_2 + imgh_2-1)
	local tyb_2 = math.ceil(cy2 - imgw_2/2 - miny + 1)
	local tye_2 = math.ceil(tyb_2 + imgw_2-1)

	local img_new_1 = torch.zeros(img_h, img_w)
	local img_new_2 = torch.zeros(img_h, img_w)

  img_new_1:sub(txb_1, txe_1, tyb_1, tye_1):copy(img1)
  img_new_2:sub(txb_2, txe_2, tyb_2, tye_2):copy(img2)
  
	local img = img_new_1:clone()
	img[img:lt(img_new_2)] = img_new_2[img:lt(img_new_2)]
	img_new= (img_new_1 + img_new_2):div(1.5)
	img[img:lt(img_new)] = img_new[img:lt(img_new)]
	img[img:gt(1)]       = 1
	img = img:div(img:max())
	
	img_new_1 = nil
	img_new_2 = nil
	collectgarbage()
	
	local flat_new_1 = torch.zeros(3,img_h, img_w)
	local flat_new_2 = torch.zeros(3,img_h, img_w)

	flat_new_1:sub(1,3, txb_1, txe_1, tyb_1, tye_1):copy(flat1)
	flat_new_2:sub(1,3, txb_2, txe_2, tyb_2, tye_2):copy(flat2)
	
	local flat = flat_new_1:clone()
	flat[flat:lt(flat_new_2)] = flat_new_2[flat:lt(flat_new_2)]
	
	flat_new_1 = nil
	flat_new_2 = nil
	
	local cimg = torch.zeros(3,img_h,img_w)
	
	return img, flat, cx, cy
	
end

function get_contours(img, flat)

  local cimg = torch.zeros(flat:size())
  local gv = img:clone():mul(5):floor()
  gv[gv:eq(1)] = 0.10
  gv[gv:eq(2)] = 0.50
  gv[gv:eq(3)] = 0.75
  gv[gv:eq(4)] = 0.90
  gv[gv:eq(5)] = 1.00
	local rv = gv:clone():pow(2):mul(-1):add(1):sqrt()/2
	cimg[1] = rv
	cimg[2] = gv
	cimg[flat:gt(cimg)] = flat[flat:gt(cimg)]
	return cimg
end

function get_scan_density(file_table, resolution, frac, dmin)
  cmb_cx      = 0
	cmb_cy      = 0
	cmb_img_h   = 5
	cmb_img_w   = 5
	cmb_density = torch.zeros(cmb_img_h, cmb_img_w)
	cmb_flat    = torch.zeros(3, cmb_img_h, cmb_img_w)

	for i = 1,#file_table do
		local srcfile = file_table[i]
		print(srcfile)
		local pc      = torch.load(srcfile)
		local img, flat, cx, cy = get_2d_density(pc, resolution, frac, dmin)
		cmb_density, cmb_flat, cmb_cx, cmb_cy = get_combined_density(cmb_density, cmb_flat, cmb_cx, cmb_cy, img, flat, cx, cy)
		srcfile = nil
		pc      = nil
		img     = nil
		flat    = nil
		cx      = nil
		cy      = nil
		collectgarbage()
		print()
	end
  collectgarbage()
  
  return cmb_density, cmb_flat, cmb_cx, cmb_cy
end

util.fs.mkdir_p(params.savdir)

file_table = util.fs.files_only(params.srcdir,'.dat')

density_img, flat_img, img_cx, img_cy = get_scan_density(file_table, params.resolution, params.sampling_fraction, params.scanner_radius)
contours = get_contours(density_img, flat_img)

image.save(params.savdir..'/density_map.png', image.combine(density_img))
image.save(params.savdir..'/walls_map.png', image.combine(flat_img))
image.save(params.savdir..'/contour_map.png', image.combine(contours))

if not params.interactive then
  collectgarbage()
  process.exit()
end


