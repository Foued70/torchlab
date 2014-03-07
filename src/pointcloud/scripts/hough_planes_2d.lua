path   = require 'path'
ffi    = require 'ffi'
ctorch = util.ctorch

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
    int hough_planes_2d(double * himg, double * xyz_map, double * intensity_map,
                 double * normal_phi, double * normal_theta,
                 double phi, double theta_res, double depth_res, 
                 double max_depth, int map_h, int map_w, 
                 double depth_thresh, double ang_thresh, double scale);
]]

local libpc   = util.ffi.load('libpcd')

cmd = torch.CmdLine()

cmd:text()
cmd:text('hough_planes')
cmd:text()
cmd:text('Options')
cmd:option(     '-arcdir', '/Users/lihui815/Documents', 'directory of arcs')
cmd:option(     '-prjdir',        'elegant-prize-3149', 'arc name')
cmd:option(     '-ptcdir',         'work/a/pointcloud', 'pointcloud dir')
cmd:option(     '-outdir',         'work/a/houghplane', 'pointcloud dir')
cmd:option(     '-swpnum',                       '005', 'sweep pointcloud basename')
cmd:option(  '-depth_res',                           5, 'hough resolution of depth')
cmd:option(  '-theta_res',                    2*pi/720, 'hough resolution of theta')
cmd:option(    '-phi_res',                    2*pi/720, 'hough resolution of phi')
cmd:option(  '-depth_thr',                          10, 'acceptance threshold for depth')
cmd:option(  '-theta_thr',                     2*pi/24, 'acceptance threshold for theta')
cmd:option(    '-phi_thr',                     2*pi/24, 'acceptance threshold for phi')
cmd:option( '-num_planes',                          10, 'number of planes to be found')
cmd:option( '-num_points',                          50, 'number of pts to count as a plane')
cmd:option(     '-scales',                           4, 'number of pts to count as a plane')
cmd:option('-interactive',                       false, 'interactive')
cmd:text()

params     = cmd:parse(process.argv)

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
  local phi   = asin(c)
  local theta = 0
  if a ~= 0 or b ~= 0 then 
    theta = acos(a/sqrt(pow(a,2) + pow(b,2)))
  end
  if b < 0 and theta ~= 0 then
    theta = -theta
  end
 return phi, theta
end

function get_normal_abc(phi, theta)
  local c   = sin(phi)
  local tmp = cos(phi)
  local b   = tmp * sin(theta)
  local a   = tmp * cos(theta)
  return a, b, c
end

function get_residual(xyz, a, b, c, d)
  return xyz[1]*a + xyz[2]*b + xyz[3]*c + d
end

function get_residual_abs(xyz, a, b, c, d)
  return get_residual(xyz, a, b, c , d):abs()
end

function get_angle_mask(nmp, a, b, c, ang_thresh, inverse)
  local num      = nmp[1]*a + nmp[2]*b + nmp[3]*c
  local den      = nmp:clone():pow(2):sum(1):squeeze():mul(a*a + b*b + c*c):sqrt()
  local cosang   = num:clone():cdiv(den)
  local msk      = cosang:acos():le(ang_thresh)
  msk[den:eq(0)] = 0
  if inverse then
    msk = msk:eq(0)
  end
  num            = nil
  den            = nil
  cosang         = nil
  collectgarbage()
  return msk
end

-- get parameters
depth_res  = params.depth_res
theta_res  = params.theta_res
phi_res    = params.phi_res
depth_thr  = params.depth_thr
theta_thr  = params.theta_thr
phi_thr    = params.phi_thr
num_planes = params.num_planes
num_points = params.num_points
scales     = params.scales

-- prep the output directory
outdir = path.join(params.arcdir, params.prjdir, params.outdir)
util.fs.mkdir_p(outdir)
outdir = path.join(outdir, '2d_'..params.swpnum..'_'..string.format('%03d', depth_res)..'_'..string.format('%03d', depth_thr)..'_'..string.format('%03d', scales)..'_'..string.format('%03d', num_planes))
util.fs.mkdir_p(outdir)

-- get the pointcloud
pcfname = path.join(params.arcdir, params.prjdir, params.ptcdir, params.swpnum..'.dat')
pc      = torch.load(pcfname)

-- get maps
xyz_map                     = pc:get_xyz_map()
depth_map                   = pc:get_depth_map()
nrm_map, nrm_phi, nrm_theta = pc:get_normal_map()
valid_mask                  = pc:get_valid_masks()
inverse_mask                = pc:get_inverse_masks()
area_map                    = pc:get_area_map()
density_map                 = pc:get_density_map()
npts_map                    = torch.ones(density_map:size()):cdiv(density_map)
npts_map[density_map:le(0)] = 0

-- get pointcloud values
max_depth    = depth_map:max()
map_h        = pc.height
map_w        = pc.width

-- get hough values
depth_dim    = floor(max_depth / depth_res + 1)
theta_dim    = floor(2 * pi / theta_res + 1)
phi_dim      = floor(pi / theta_res + 1)

-- prepare accumulation maps
plane_nrm  = torch.zeros(3,map_h,map_w)
plane_nphi = torch.zeros(map_h, map_w)
plane_nthe = torch.zeros(map_h, map_w)
plane_wgt  = torch.zeros(map_h,map_w)
plane_cmp  = torch.zeros(3,map_h,map_w)
leftoverm  = valid_mask:clone()

-- prepare colors
colormap   = image.colormap(2*3*scales*num_planes)

-- initialize plane tensor
plane_eqs = torch.Tensor(2*3*scales*num_planes, 4)

-- initialize counters
plane_num  = 0
faild_num = 0
color_num  = 0

-- save prelim maps
image.save(outdir..'/area_'..'_'..string.format('%03d',0)..'.png', image.combine(area_map))
image.save(outdir..'/npts_'..'_'..string.format('%03d',0)..'.png', image.combine(npts_map))
image.save(outdir..'/left_'..'_'..string.format('%03d',0)..'.png', image.combine(leftoverm))

for p = 1,3 do

	local dirp = outdir..'/'..(p)
	util.fs.mkdir_p(dirp)

	local phi  = (p-2) * pi / 2

	local iter = num_planes

	local area_tmp = area_map:clone()
	local npts_tmp = npts_map:clone()
	local msk = torch.zeros(leftoverm:size())
	local visited_planes = torch.zeros(theta_dim,depth_dim):byte()

	if phi == 0 then
		msk = nrm_phi:clone():abs():gt(pi/3)
		it = num_vplanes
	elseif phi < 0 then
		msk = nrm_phi:gt(-pi/4)
	else
		msk = nrm_phi:lt(pi/4)
	end

	area_tmp[msk] = 0
	npts_tmp[msk] = 0
	msk       = nil
	collectgarbage()
	
	image.save(outdir..'/area_'..string.format('%03d',p)..'_'..string.format('%02d', 0)..'.png', image.combine(area_tmp))
	image.save(outdir..'/npts_'..string.format('%03d',p)..'_'..string.format('%02d', 0)..'.png', image.combine(npts_tmp))
	
	for s = 1,scales do
		
		plane_num = 0
		faild_num = 0

		while (max(plane_num, faild_num) < 2*iter) and ((area_tmp + npts_tmp):sum() > 0) do
	
			local intensity = area_tmp:clone()
			if plane_num % 2 == 0 then
				intensity = npts_tmp:clone()
			end
		
			local himg  = torch.zeros(theta_dim, depth_dim):clone():contiguous()
		
		  --[[
			libpc.hough_planes_2d(torch.data(himg), torch.data(xyz_map), torch.data(intensity),
														torch.data(nrm_phi:clone()), torch.data(nrm_theta:clone()),
														phi, theta_res, depth_res, max_depth, map_h, map_w,
														10, pi*3/8, 1/(scales-s+1))
      --[[]]
      msk = torch.rand(himg:size()):le(1/(scales-s+1)):double()
      libpc.hough_planes_2d(torch.data(himg), torch.data(xyz_map), torch.data(intensity:clone():cmul(msk)),
														torch.data(nrm_phi:clone()), torch.data(nrm_theta:clone()),
														phi, theta_res, depth_res, max_depth, map_h, map_w,
														10, pi/3, 1)
      msk = nil
      collectgarbage()
      --[[]]
	
			if (plane_num + faild_num) > 0 then
				himg[visited_planes] = -1
			end			
		
			local max_val, max_ind = himg:clone():reshape(theta_dim * depth_dim):max(1)
			local max_val = max_val[1]
			local max_ind = max_ind[1]
		
			local max_theta = ceil(max_ind/depth_dim)
			local max_dist  = max_ind - ((max_theta - 1) * depth_dim)
			if phi == 0 then
				visited_planes[max_theta][max_dist] = 1
			else
				visited_planes:sub(1, theta_dim, max_dist, max_dist):fill(1)
			end

			local theta = (max_theta-1)*theta_res-pi
			local depth = (max_dist-1)*depth_res
			print('scale, p, plane_num, faild_num, color_num: ', s, p, plane_num, faild_num, color_num)
			print('num_int, maxvals: ', intensity:gt(0):double():sum(), himg:max(), max_val, himg[max_theta][max_dist])
			print('th_ind, dp_ind: ', max_theta, max_dist) 
			print('phi, theta, depth: ', phi, theta, depth)
		
			local a, b, c = get_normal_abc(phi, theta)
			local d = depth
		
			print('a_0, b_0, c_0, d_0: ', a,b,c,d)
		
			local rsd        = torch.zeros(map_h, map_w)
			local plane_mask = torch.zeros(map_h, map_w):byte()
			local wig        = 0
			
			local rsd = get_residual_abs(xyz_map, a, b, c, d)
      local msk = get_angle_mask(nrm_map, a, b, c, pi/2)
			local plane_mask = rsd:le(depth_thr):cmul(msk):cmul(leftoverm):cmul(intensity:gt(0))
		
			print('plane points 0: ', plane_mask:double():sum())
		
			-- refit the plane to the points found
			if plane_mask:double():sum() > 2 then
		
				a,b,c,d = refit_plane(xyz_map, plane_mask, a, b, c, d)
		
				print('a_1, b_1, c_1, d_1: ', a,b,c,d)
		
				phi,theta = get_normal_phth(a,b,c)
				depth     = d
		
				print('phi, theta, depth: ', phi, theta, depth)
		
				rsd = get_residual_abs(xyz_map, a, b, c, d)
				msk = get_angle_mask(nrm_map, a, b, c, pi/2)
				plane_mask = rsd:le(depth_thr):cmul(msk)
			
				print('plane points 1: ', plane_mask:double():sum())
				if plane_mask:double():sum() > 0 then
		
					local msk = get_angle_mask(nrm_map, a, b, c, pi/3, true)
					if msk:double():sum() > 0 then
						plane_mask[msk] = 0
					end
					msk = nil
					collectgarbage()
	
					print('plane points 2: ', plane_mask:double():sum())
					if plane_mask:double():sum() > num_points then
				
						plane_nmp, plane_phi, plane_theta, plane_rsd, plane_mask = pc:get_normal_map(true,50,nil,plane_mask)
				
						print('plane points 3: ', plane_mask:double():sum())
						if plane_mask:double():sum() > num_points then
					
							local msk = get_angle_mask(plane_nmp, a, b, c, pi/8, true)
							if msk:double():sum() > 0 then
								plane_mask[msk] = 0
							end
							msk = nil
							collectgarbage()
					
							print('plane points 4: ', plane_mask:double():sum())
							if plane_mask:double():sum() > num_points then
						
								plane_nmp, plane_phi, plane_theta, plane_rsd, plane_mask = pc:get_normal_map(true,100,nil,plane_mask)
							
								print('plane points 5: ', plane_mask:double():sum())
								if plane_mask:double():sum() > num_points then
							
									local msk = get_angle_mask(plane_nmp, a, b, c, pi/12, true)
									if msk:double():sum() > 0 then
										plane_mask[msk] = 0
									end
									msk = nil
									collectgarbage()
						
									print('plane points 6: ', plane_mask:double():sum())
									if plane_mask:double():sum() > num_points then
									
										local wgt = rsd:clone():mul(-1):add(1.25*depth_thr):cmul(plane_mask:double())[plane_mask]
										plane_nphi[plane_mask] = plane_nphi[plane_mask] + wgt:clone():mul(phi)
										plane_nthe[plane_mask] = plane_nthe[plane_mask] + wgt:clone():mul(theta)
										plane_wgt[plane_mask]  = plane_wgt[plane_mask]  + wgt:clone()
										wgt = nil
										collectgarbage()
							
							      print('plane points 7: ', plane_mask:clone():cmul(leftoverm):double():sum())
							      if plane_mask:clone():cmul(leftoverm):double():sum() > num_points/10 then
							      
  							      cmp_local  = torch.zeros(3,map_h,map_w)
	  									plane_num = plane_num+1
		  								color_num = color_num+1
			  							cmap = colormap[color_num]
										
				  						for dim = 1,3 do
					  						plane_cmp[dim][plane_cmp[dim]:eq(0):cmul(plane_mask)] = cmap[dim]
						  					cmp_local[dim][plane_mask] = cmap[dim]
							  			end
							
								  		if plane_num % 10 == 0 then
									  		image.save(dirp..'/plane_glb_'..string.format('%02d',s)..'_'..plane_num..'.png',image.combine(plane_cmp))
										  end
								
						  				image.save(dirp..'/plane_mask_'..string.format('%02d',s)..'_'..plane_num..'.png',image.combine(plane_mask:double()))
							  			plane_eqs[color_num] = torch.Tensor({a,b,c,d})
							  			
							  			leftoverm[plane_mask] = 0
							  			area_tmp[plane_mask] = 0
  										npts_tmp[plane_mask] = 0
							  		else
							  		  faild_num = faild_num + 0.25
							  		end
											
										pnrm = nil
										rsd  = nil
										min  = nil
										wgt  = nil
										cmp_local = nil
										collectgarbage()
							
									else
										faild_num = faild_num + 1
									end
								else
							
									faild_num = faild_num + 1
								end
			
							else
								faild_num = faild_num + 1
							end
	
						else
							faild_num = faild_num + 1
						end
					
						plane_nmp   = nil
						plane_phi   = nil
						plane_theta = nil
						plane_rsd   = nil
						collectgarbage()
			
					else
						faild_num = faild_num + 1
					end
			
				else
					faild_num = faild_num + 1
				end
			
			else
				faild_num = faild_num + 1
			end
		
			pc:get_normal_map(true)
		
			himg        = nil
			rsd         = nil
			plane_mask  = nil
			collectgarbage()
			print()
	
		end
		collectgarbage()
	
		image.save(outdir..'/area_'..string.format('%03d',p)..'_'..string.format('%02d',s)..'.png', image.combine(area_tmp))
		image.save(outdir..'/npts_'..string.format('%03d',p)..'_'..string.format('%02d',s)..'.png', image.combine(npts_tmp))
		image.save(outdir..'/left_'..string.format('%03d',p)..'_'..string.format('%02d',s)..'.png', image.combine(leftoverm))
		image.save(outdir..'/planes_'..string.format('%03d',p)..'_'..string.format('%02d',s)..'.png', image.combine(plane_cmp))
	
	end
	
	area_tmp = nil
	npts_tmp = nil
	visited_planes = nil
	collectgarbage()
		
end

collectgarbage()

plane_nphi:cdiv(plane_wgt)
plane_nthe:cdiv(plane_wgt)
pmsk = plane_wgt:le(0)

plane_nrm[3] = plane_nphi:clone():sin()
plane_nrm[2] = plane_nphi:clone():cos():cmul(plane_nthe:clone():sin())
plane_nrm[1] = plane_nphi:clone():cos():cmul(plane_nthe:clone():cos())

plane_nrm[pmsk:repeatTensor(3,1,1)] = 0

torch.save(outdir..'/plane_eqs.dat', plane_eqs)
image.save(outdir..'/plane_all.png', image.combine(plane_cmp))
image.save(outdir..'/mask_all.png', pmsk:double())
image.save(outdir..'/nmp_planes.png', image.combine(plane_nrm))
image.save(outdir..'/nmp_phi.png', image.combine(plane_nphi))
image.save(outdir..'/nmp_theta.png', image.combine(plane_nthe))

pc:get_normal_map(true)

if pmsk:double():sum() > 0 then
  
  plane_nrm = plane_nrm + pc.normal_map:cmul(pmsk:double():repeatTensor(3,1,1))
  
end

pnmp = nil
pmsk = nil
nmsk = nil
tmp  = nil
collectgarbage()

image.save(outdir..'/nmp_all.png', image.combine(plane_nrm))

pc:get_normal_map(true)
image.save(outdir..'/nmp_orig.png', image.combine(pc.normal_map))


if not params.interactive then
  collectgarbage()
  process.exit()
end
