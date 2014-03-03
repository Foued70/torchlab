loader = pointcloud.loader
ffi    = require 'ffi'
ctorch = util.ctorch
path   = require 'path'

pi     = math.pi
cos    = math.cos
sin    = math.sin
acos   = math.acos
asin   = math.asin
max    = math.max
min    = math.min
abs    = math.abs
pow    = math.pow
sqrt   = math.sqrt
floor  = math.floor
ceil   = math.ceil

ffi.cdef
[[
    int hough_planes(double * himg, double * xyz_map, double * intensity_map,
                 double phi, double theta_res, double depth_res, 
                 double max_depth, int map_h, int map_w);
]]

local libpc   = util.ffi.load('libpcd')


cmd = torch.CmdLine()

cmd:text()
cmd:text('hough_planes')
cmd:text()
cmd:text('Options')
cmd:option('-arcdir', '/Users/lihui815/Documents', '')
cmd:option('-prjdir', 'elegant-prize-3149', '')
cmd:option('-orgdir', 'source/po_scan/a', '')
cmd:option('-swpdir', '005', 'sweep pointcloud directory')
cmd:option('-opset',   0,   'set')
cmd:option('-numpt', 100, 'ratio of pts to count as a plane')
cmd:option('-interactive', false, 'ratio of pts to count as a plane')
cmd:text()

params     = cmd:parse(process.argv)

sets = {}
sets[0] = {10,  360, 100, 10,  25}
sets[1] = { 5,  720,  50, 25,  50}
--sets[2] = { 5,  720,  25, 35,  75}
sets[2] = { 5,  720,  25, 35,  0}
sets[3] = { 5,  720,  15, 50, 100}
sets[4] = { 2, 1440,  50, 30, 100}
sets[5] = { 2, 1440,  25, 50, 150}
sets[6] = { 2, 1440,  10, 75, 200}
sets[6] = { 5,  720,  25, 20,   0}

arcdir = params.arcdir
prjdir = params.prjdir
orgdir = params.orgdir
swpdir = params.swpdir
srcdir = path.join(arcdir,prjdir,orgdir, swpdir)
thresh = params.thresh
opset  = params.opset
numpt  = params.numpt 

set_u = sets[opset]

function refit_plane(xyz_map, xyz_mask, 
                     a_init, b_init, c_init, d_init)
  
  local p0    = torch.Tensor({a_init,b_init,c_init})*(-d_init)
  
  local numpts = xyz_mask:double():sum()
  local pts    = torch.zeros(3,numpts)
  
  for dim = 1,3 do
    pts[dim] = xyz_map[dim][xyz_mask] - p0[dim]
  end
  
  if numpts > 25000 then
    pts = pts:sub(1,3,1,25000):clone()
  end
  
  local mat = torch.svd(pts)
  local nrm = mat:select(2,3)
  local d_fin = -(p0:clone():cmul(nrm):sum())
  
  if d_fin < 0 then
    nrm = nrm:mul(-1)
    d_fin = -(p0:clone():cmul(nrm):sum())
  end
  
  a_fin = nrm[1]
  b_fin = nrm[2]
  c_fin = nrm[3]
  
  p0  = nil
  pts = nil
  mat = nil
  nrm = nil
  collectgarbage()
  
  return a_fin, b_fin, c_fin, d_fin
  
end

function get_normal_phth(a, b, c)
  if type(a) == 'number' then
    local phi   = asin(c)
    local theta = 0
    if a ~= 0 or b ~= 0 then 
      theta = acos(a/sqrt(pow(a,2) + pow(b,2)))
    end
    if b < 0 and theta ~= 0 then
      theta = -theta
    end
    return phi, theta
  else
    local phi    = c:clone():asin()
    local theta  = a:clone():cdiv(a:clone():pow(2):add(b:clone():pow(2)):sqrt()):acos()
    local mask  = a:eq(0):cmul(b:eq(0))
    theta[mask] = 0
    mask:cmul(b:lt(0))
    theta[mask] = -theta[mask]
    mask = nil
    collectgarbage()
    return phi, theta
  end
end

_G.pc  = loader.load_pobot_ascii(srcdir)

xyz_map      = pc:get_xyz_map()
depth_map    = pc:get_depth_map()
max_depth    = pc.xyz_radius:max()
depth_res    = set_u[1]
theta_res    = pi/set_u[2]
rsd_res      = set_u[3]
num_hplanes  = set_u[4]
num_vplanes  = set_u[5]
dir          = 'data/'..prjdir..'_'..swpdir..'_'..opset


depth_dim    = floor(max_depth / depth_res + 1)
theta_dim    = floor(2 * pi / theta_res + 1)
map_h        = pc.height
map_w        = pc.width
dense_map    = pc:get_density_map()
npts         = torch.ones(map_h,map_w):cdiv(dense_map)
npts[dense_map:le(0)] = 0

util.fs.mkdir_p(dir)

nrm_map = pc:get_normal_map()
xyz_nrm = xyz_map:clone():norm(2,1)
xyz_nnn = xyz_map:clone():cdiv(xyz_nrm:repeatTensor(3,1,1))
xyz_nnn[xyz_nrm:le(0):repeatTensor(3,1,1)] = 0
dotp = nrm_map:clone():cmul(xyz_nnn):sum(1):abs():squeeze()
dotp:pow(2):mul(-1):add(1)
dotp[dotp:gt(1)] = 1
dotp[dotp:lt(0)] = 0
dotp:sqrt()

dpt_xyz = xyz_map:sub(1,2):clone():norm(2,1):squeeze()
r1      = dpt_xyz:clone():mul(2*pi/pc.width)/10
r2      = depth_map:clone():mul((pc.xyz_phi_map:max() - pc.xyz_phi_map:min())/pc.height)/10
area = r1:clone():cmul(r2):mul(pi):cmul(dotp)

image.save(dir..'/area.png', image.combine(area))
image.save(dir..'/npts.png', image.combine(npts))

visited_planes = torch.zeros(theta_dim,depth_dim):byte()

plane_nrm = torch.zeros(3,map_h,map_w)
plane_wgt = torch.zeros(3,map_h,map_w)
plane_cmp = torch.zeros(3,map_h,map_w)
leftoverm = pc:get_valid_masks()
 
colormap = image.colormap(2*(2*num_hplanes+num_vplanes))
fnum     = 0
cnum     = 0

image.save(dir..'/left_beg.png',leftoverm:double())

plane_eqs = {}

for p = 0,2 do

  dirp = dir..'/'..(p)
  dirf = dirp..'/fake'
  util.fs.mkdir_p(dirp)
  util.fs.mkdir_p(dirf)
  
  phi = (p-1) * pi/2
  
  it = num_hplanes
  
  intensity1 = area:clone()
  intensity2 = npts:clone()
  local msk = torch.zeros(intensity1:size())
  
  if p == 1 then
    msk = pc.normal_phi_map:clone():abs():gt(pi/3)
    it = num_vplanes
  elseif p == 0 then
    msk = pc.normal_phi_map:gt(-pi/3)
  else
    msk = pc.normal_phi_map:lt(pi/3)
  end
  
  intensity1[msk] = 0
  intensity2[msk] = 0
  
  image.save(dir..'/area_'..p..'_1.png', image.combine(intensity1))
  image.save(dir..'/npts_'..p..'_1.png', image.combine(intensity2))
  
  i = 0
  fnum = 0
	while (max(i,fnum) < 2*it) and ((intensity1 + intensity2):sum() > 0) do
	
	  intensity = intensity1:clone()
	  if i % 2 == 0 then
	    intensity = intensity2:clone()
	  end
	  
	  himg  = torch.zeros(theta_dim, depth_dim):clone():contiguous()
	  
		libpc.hough_planes(torch.data(himg), torch.data(xyz_map), torch.data(intensity),
										 phi, theta_res, depth_res, 
										 max_depth, map_h, map_w)
	
		if (i + fnum) > 0 then
			himg[visited_planes] = -1
		end			
		
		max_val, max_ind = himg:clone():reshape(theta_dim * depth_dim):max(1)
		max_val = max_val[1]
		max_ind = max_ind[1]
		
		max_theta = ceil(max_ind/depth_dim)
		max_dist  = max_ind - ((max_theta - 1) * depth_dim)
		if p == 1 then
  		--visited_planes:sub(max(max_theta - 2, 1), min(max_theta + 2, theta_dim), max(max_dist - 2, 1), min(max_dist + 2, depth_dim)):fill(1)
  		visited_planes[max_theta][max_dist] = 1
  	else
  	  --visited_planes:sub(1, theta_dim, max(max_dist - 2, 1), min(max_dist + 2, depth_dim)):fill(1)
  	  visited_planes:sub(1, theta_dim, max_dist, max_dist):fill(1)
  	end

    phi = (p-1) * pi/2
		theta = (max_theta-1)*theta_res-pi
		depth = (max_dist-1)*depth_res
		print('p, i, fnum, cnum: ', p, i, fnum, cnum)
		print('num_int, maxvals: ', intensity:gt(0):double():sum(), himg:max(), max_val, himg[max_theta][max_dist])
		print('th_ind, dp_ind: ', max_theta, max_dist) 
		print('phi, theta, depth: ', phi, theta, depth)
		
		a = cos(theta)*cos(phi)
    b = sin(theta)*cos(phi)
	  c = sin(phi)
		d = depth
		
		print('a_0, b_0, c_0, d_0: ', a,b,c,d)
		
		rsd = (xyz_map[1]*a + xyz_map[2]*b + xyz_map[3]*c + d):abs()

		plane_mask = rsd:le(rsd_res):cmul(leftoverm)
		if p == 1 then
		  plane_mask:cmul(pc.normal_phi_map:clone():abs():lt(pi/3))
		else
			plane_mask:cmul(pc.normal_phi_map:clone():abs():gt(pi/3))
		end
		
		print('plane points 0: ', plane_mask:double():sum())
		if plane_mask:double():sum() > 2 then
		
			a,b,c,d = refit_plane(xyz_map, plane_mask, a, b, c, d)
		
			print('a_1, b_1, c_1, d_1: ', a,b,c,d)
		
			phi,theta = get_normal_phth(a,b,c)
			depth     = d
		
			print('phi, theta, depth: ', phi, theta, depth)
		
			plane_mask = nil
			collectgarbage()
		
			plane_mask = torch.zeros(map_h,map_w):byte()
			thres_mask = torch.zeros(map_h,map_w):byte()

			for ph_wig = -1,1 do
			
				phi_f   = phi    + ph_wig*theta_res
				if (phi_f <= pi/2) and (phi_f >= -pi/2) then
			
					for d_wig = -1,1 do
				
						depth_f = depth + d_wig *depth_res
						if (depth_f > 0) then
				
							for th_wig = -1,1 do
					

								theta_f = theta + th_wig*theta_res
					
								a = cos(theta_f)*cos(phi_f)
								b = sin(theta_f)*cos(phi_f)
								c = sin(phi_f)
								d = depth_f
		
								rsd = (xyz_map[1]*a + xyz_map[2]*b + xyz_map[3]*c + d):abs()

								plane_mask = (plane_mask + rsd:le(rsd_res)):gt(0)
								thres_mask = (thres_mask + rsd:le(rsd_res)):gt(0)
						
								rsd = nil
								collectgarbage()
							end
						end
					end
				end
			end
							
			print('plane points 1: ', plane_mask:double():sum())
			if plane_mask:double():sum() > 0 then
		
				if p == 1 then
					plane_mask:cmul(pc.normal_phi_map:clone():abs():lt(pi/3))
				else
					plane_mask:cmul(pc.normal_phi_map:clone():abs():gt(pi/3))
				end
	
				print('plane points 2: ', plane_mask:double():sum())
				if plane_mask:double():sum() > numpt then
					thres_mask:cmul(plane_mask)
				
					plane_nmp, plane_phi, plane_theta, plane_rsd, plane_mask = pc:get_normal_map(true,50,nil,plane_mask)
				
					print('plane points 3: ', plane_mask:double():sum())
					if plane_mask:double():sum() > numpt then
						thres_mask:cmul(plane_mask)
	
						local msk = plane_phi:clone():abs():lt(pi*3/8)
						if p == 1 then
							local ang_thresh = pi/8
							local theta_diff = (plane_theta - theta):abs()
							local theta_mask = theta_diff:gt(ang_thresh):cmul(theta_diff:lt(2*pi - ang_thresh))
							local phi_mask   = plane_phi:clone():abs():gt(ang_thresh)
							msk              = (theta_mask + phi_mask):gt(0)
							theta_diff       = nil
							theta_mask       = nil
							phi_mask         = nil
							collectgarbage()
						end
						if msk:double():sum() > 0 then
							plane_mask[msk] = 0
						end
						msk = nil
						collectgarbage()
					
						print('plane points 4: ', plane_mask:double():sum())
						if plane_mask:double():sum() > numpt then
							thres_mask:cmul(plane_mask)
						
							plane_nmp, plane_phi, plane_theta, plane_rsd, plane_mask = pc:get_normal_map(true,100,nil,plane_mask)
	
							local msk = plane_phi:clone():abs():lt(5*pi/12)
							if p == 1 then
								local ang_thresh = pi/12
								local theta_diff = (plane_theta - theta):abs()
								local theta_mask = theta_diff:gt(ang_thresh):cmul(theta_diff:lt(2*pi - ang_thresh))
								local phi_mask   = plane_phi:clone():abs():gt(ang_thresh)
								msk              = (theta_mask + phi_mask):gt(0)
								theta_diff       = nil
								theta_mask       = nil
								phi_mask         = nil
							end
							if msk:double():sum() > 0 then
								plane_mask[msk] = 0
							end
							msk = nil
							collectgarbage()
						
							print('plane points 5: ', plane_mask:double():sum())
							if plane_mask:double():sum() > numpt then
								thres_mask:cmul(plane_mask)
							
								cmp_local  = torch.zeros(3,map_h,map_w)
								i = i+1
								cnum = cnum+1
								cmap = colormap[cnum]
							
								a = cos(theta)*cos(phi)
								b = sin(theta)*cos(phi)
								c = sin(phi)
								d = depth
											
								plane_nrm[1][plane_mask] = plane_nrm[1][plane_mask] + a
								plane_nrm[2][plane_mask] = plane_nrm[2][plane_mask] + b
								plane_nrm[3][plane_mask] = plane_nrm[3][plane_mask] + c
							
								plane_wgt:add(plane_mask:double():repeatTensor(3,1,1))
							
								for dim = 1,3 do
									plane_cmp[dim][plane_cmp[dim]:eq(0):cmul(plane_mask)] = cmap[dim]
									cmp_local[dim][plane_mask] = cmap[dim]
								end
							
								if i % 10 == 0 then
									image.save(dirp..'/plane_glb_'..i..'.png',image.combine(plane_cmp))
								end
								
								--[[]]
								local pnrm = torch.zeros(plane_nrm:size())
								pnrm[1][plane_mask] = a
								pnrm[2][plane_mask] = b
								pnrm[3][plane_mask] = c
								pnrm:sub(1,3,1,1,1,1):copy(torch.Tensor({1,1,1}))
								pnrm:sub(1,3,1,1,2,2):copy(torch.Tensor({-1,-1,-1}))
								image.save(dirp..'/plane_mask_'..i..'.png',image.combine(plane_mask:double()))
								plane_eqs[cnum] = torch.Tensor({a,b,c,d})
								--[[]]
								
								--image.save(dirp..'/plane_lcl_'..i..'.png',image.combine(cmp_local))
								
				
				        pnrm = nil
								rsd  = nil
								min  = nil
								wgt  = nil
								cmp_local = nil
								collectgarbage()
							
							else
								fnum = fnum + 1
							end
			
						else
							fnum = fnum + 1
						end
	
					else
						fnum = fnum + 1
					end
			
				else
					fnum = fnum + 1
				end
			
				leftoverm[thres_mask] = 0
				intensity1[thres_mask] = 0
				intensity2[thres_mask] = 0
				--[[
				image.save(dirp..'/area_'..i..'_'..fnum..'.png',image.combine(intensity1))
				image.save(dirp..'/npts_'..i..'_'..fnum..'.png',image.combine(intensity2))
				--[[]]
			
			else
			  fnum = fnum + 1
      end
      
    else
      fnum = fnum + 1
    end
    
		pc:get_normal_map(true)
	
		
		max_val     = nil
		rsd         = nil
		plane_nmp   = nil
		plane_phi   = nil
		plane_theta = nil
		plane_rsd   = nil
		plane_mask  = nil
		thres_mask  = nil
		himg        = nil
		collectgarbage()
		print()
	
	end
	collectgarbage()
	
	image.save(dir..'/plane_glb_'..p..'.png',image.combine(plane_cmp))
	image.save(dir..'/left_end_'..p..'.png',leftoverm:double())
	image.save(dir..'/area_'..p..'_2.png', image.combine(intensity1))
  image.save(dir..'/npts_'..p..'_2.png', image.combine(intensity2))
  
  intensity1 = nil
  intensity2 = nil
	
end
collectgarbage()

plane_nrm:cdiv(plane_wgt)
pmsk = plane_wgt:gt(0)
plane_nrm:cdiv(plane_nrm:clone():norm(2,1):repeatTensor(3,1,1))

pmsk = plane_wgt:le(0)
plane_nrm[pmsk] = 0

torch.save(dir..'/plane_eqs.dat', plane_eqs)
image.save(dir..'/plane_all.png', image.combine(plane_cmp))
image.save(dir..'/mask_all.png', pmsk:double())
image.save(dir..'/nmp_planes.png', image.combine(plane_nrm))

pc:get_normal_map(true)

if pmsk:double():sum() > 0 then
  
  plane_nrm = plane_nrm + pc.normal_map:cmul(pmsk:double())
  
end

pnmp = nil
pmsk = nil
nmsk = nil
tmp  = nil
collectgarbage()

image.save(dir..'/nmp_all.png', image.combine(plane_nrm))

pc:get_normal_map(true)
image.save(dir..'/nmp_orig.png', image.combine(pc.normal_map))


if not params.interactive then
  collectgarbage()
  process.exit()
end