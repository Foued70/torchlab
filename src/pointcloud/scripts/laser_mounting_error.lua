path = require 'path'

cmd = torch.CmdLine()

cmd:text()
cmd:text('laser mounting error')
cmd:text()
cmd:text('Options')
cmd:option(     '-arcdir', '/Users/lihui815/Documents', 'directory of arcs')
cmd:option(     '-prjdir',        'elegant-prize-3149', 'arc name')
cmd:option(     '-ptcdir',         'work/a/pointcloud', 'pointcloud dir')
cmd:option(     '-outdir',         'work/a/pointcloud_fixed', 'pointcloud dir')
cmd:option(     '-plydir',         'work/a/ply',              'pointcloud dir')
cmd:option(     '-pfxdir',         'work/a/ply_fixed',              'pointcloud dir')
cmd:option('-interactive',                       false, 'interactive')
cmd:text()

params   = cmd:parse(process.argv)

function find_average_heights(files, brows, phi_0)

	local numfs    = #files

	local avg_bz   = torch.range(1,brows):repeatTensor(numfs,1):double()

	for i = 1, numfs do

		local pcfname  = files[i]
		local pc       = torch.load(pcfname)

    local height          = pc.height
		local dpt             = pc:get_depth_map()
		local xyz, phi, theta = pc:get_xyz_map()
		local vmsk            = pc:get_valid_masks()
		local nm, nphi        = pc:get_normal_map()
		
		nm = nil
		pc = nil
		collectgarbage()
	
		phi = phi + phi_0
		local cosphi = phi:clone():cos()
		local sinphi = phi:clone():sin()
		local costhe = theta:clone():cos()
		local sinthe = theta:clone():sin()
		xyz[1]       = dpt:clone():cmul(cosphi):cmul(costhe)
		xyz[2]       = dpt:clone():cmul(cosphi):cmul(sinthe)
		xyz[3]       = dpt:clone():cmul(sinphi)
		xyz[vmsk:eq(0):repeatTensor(3,1,1)] = 0
		
		cosphi = nil
		sinphi = nil
		costhe = nil
		sinthe = nil
		phi    = nil
		theta  = nil
		dpt    = nil
		collectgarbage()

		local bot_v    = vmsk:clone():cmul(nphi:gt( math.pi*3/8)):sub(height-brows+1,height):clone()
		local bot_z    = xyz:sub(3,3,height-brows+1,height):clone():abs():squeeze()
		local bot_c    = bot_z[brows][bot_v[brows]]:clone():mean()
		bot_v:cmul((bot_z - bot_c):abs():lt(50))
		
		bot_c    = bot_z[brows][bot_v[brows]]:clone():mean()

		avg_bz[i]:apply(function(c)
									 return (bot_c - bot_z[c][bot_v[c]]:clone():mean())
								 end)
	
		xyz    = nil
		vmsk   = nil
		bot_v  = nil
		bot_z  = nil
		bot_c  = nil
	  collectgarbage()
	  
	end
	
	collectgarbage()
	return avg_bz
end

function print_statement(avg_bz, brows, phi_0)
  print('phi_0: ', phi_0)
  print()
  
  local avg_bz_mean = avg_bz:mean(1)
  local mean = avg_bz_mean:mean()
  print(avg_bz_mean)
  print('mean bot: ', mean)
  
  mean = 0
  local stdv = math.sqrt((avg_bz_mean-mean):pow(2):mean())
  local skew = (avg_bz_mean-mean):pow(3):div(math.pow(stdv,3)):mean()
  print('stdv bot: ', stdv)
  print('skew bot: ', skew)
  
  print()
  print()
end

function fix_all(files, phi_0, outdir, plydir, pfxdir)
  util.fs.mkdir_p(outdir)
  util.fs.mkdir_p(plydir)
  util.fs.mkdir_p(pfxdir)
  local numfs    = #files
	for i = 1, numfs do
	  local pname = files[i]
	  local bname = path.basename(pname, '.dat')
	  local pc    = torch.load(pname)
	  pc:save_to_ply_file(path.join(plydir, bname..'.ply'), false, false, false, true)
	  pc:fix_phi(phi_0)
	  pc:save_to_data_file(path.join(outdir, bname..'.dat'))
	  pc:save_to_ply_file(path.join(pfxdir, bname..'.ply'), false, false, false, true)
	  pc = nil
	  collectgarbage()
	end
end

brows    = 25
pcdir    = path.join(params.arcdir, params.prjdir, params.ptcdir)
files    = util.fs.files_only(pcdir, '.dat')
outdir   = path.join(params.arcdir, params.prjdir, params.outdir)
plydir   = path.join(params.arcdir, params.prjdir, params.plydir)
pfxdir   = path.join(params.arcdir, params.prjdir, params.pfxdir)

--[[]]
phi_0  = 0
step   = 0.01
accept = 0.001
avg_bz = find_average_heights(files, brows, phi_0)
avg_bz_mean = avg_bz:mean(1)
mean = avg_bz_mean:mean()
score = mean
dir = (score > 0)

print_statement(avg_bz, brows, phi_0)

while math.abs(score) > accept do

  if dir then
    phi_0 = phi_0 - step
  else
    phi_0 = phi_0 + step
  end
  
  avg_bz = find_average_heights(files, brows, phi_0)
  avg_bz_mean = avg_bz:mean(1)
  mean = avg_bz_mean:mean()
  score = mean
  
  print_statement(avg_bz, brows, phi_0)
  
  dir_new = (score > 0)
  if (dir_new ~= dir) then
   step = step * (1/3)
  end
  dir = dir_new
  
end

fix_all(files, phi_0, outdir, plydir, pfxdir)

if not params.interactive then
  collectgarbage()
  process.exit()
end