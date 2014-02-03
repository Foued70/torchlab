local ufs = util.fs
local path = require 'path'
local fs = require 'fs'
local ptcld = PointCloud.PointCloud
local log = require '../util/log'
local io = require 'io'
local saliency = require '../image/saliency'
local imgraph = require '../imgraph'
local ffi = require 'ffi'
local ctorch = util.ctorch -- ctorch needs to be loaded before we reference THTensor stuff in a cdef

print(1)


ffi.cdef
[[
    int compute_number_of_plane_points(double* result, double* xyz, char* rmask,
                                   double* nmp, double* ndd, 
                                   double res_thresh, int height, int width);
                                   
    int compute_seed_scores(double* result, double* xyz,
                        double* nmp, double* ndd, 
                        double res_thresh, int len);

]]

local libpc   = util.ffi.load('libpointcloud')

--[[]]

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('test')
cmd:text()
cmd:text('Options')
cmd:option('-arcs_dir','/Users/lihui815/Documents') --'/Users/lihui815/cloudlab/build/usr/local/tmp/arcs'

--[[
cmd:option('-proj_dir','temporary-circle-6132') --'temporary-circle-6132' --'motor-unicorn-0776'
--[[]]
cmd:option('-proj_dir','motor-unicorn-0776') --'temporary-circle-6132' --'motor-unicorn-0776'
--[[]]

cmd:option('-sweep_dir','sweep_001')

cmd:option('-non_interactive',false)
cmd:text()

function get_best_seed(seed_scores)
  if seed_scores:max() > 0 then
    --[[]]
    local height = seed_scores:size(1)
    local width = seed_scores:size(2)
    local len = height*width
    local ssr = seed_scores:reshape(len):clone()
    local m,i = ssr:max(1)
    local ind = i[1]
    local w = ((ind-1 + width) % width) + 1
    local h = (ind-w)/width + 1
    return h,w
    --[[]]
    
    --[[
    local m,i,h,w
    m = seed_scores:max(1):squeeze()
    m,i = m:max(1)
    print(m[1],i[1])
    w = i[1]
    m = seed_scores:max(2):squeeze()
    m,i = m:max(1)
    h = i[1]
    print(m[1],i[1])
    return h,w
    --[[]]
    
  end
end

function get_seed_eqn(nmp,ndd,seed_h,seed_w)
  if (seed_h and seed_w) then
    local eqn = torch.zeros(4)
    eqn[4] = -ndd[seed_h][seed_w]
    eqn:sub(1,3):copy(nmp:select(2,seed_h):select(2,seed_w):clone():contiguous())
    return eqn
  end
end

function get_plane_points(xyz,nmp,eqn,res_thresh,norm_thresh)
  
  local height = xyz:size(2)
  local width = xyz:size(3)
  local xyz1 = torch.ones(4,height,width)
  xyz1:sub(1,3):copy(xyz:clone())
  
  local eqn_plane = eqn:repeatTensor(width,height,1):transpose(1,3):clone():contiguous()
  local res = eqn_plane:clone():cmul(xyz1):sum(1):squeeze():abs()
  
  eqn_plane = eqn_plane:sub(1,3):clone()
  local nrm = (eqn_plane:clone():cmul(nmp):sum(1):abs()):cdiv(
              (eqn_plane:norm(2,1):cmul(nmp:norm(2,1)))):squeeze():acos()
  
  local pp = res:lt(res_thresh):cmul(
             nrm:lt(norm_thresh))
             
  --todo filter pp by connectedness to the seed
  
  return pp
end

function fit_plane_to_points_and_seed(points, seed)
  -- points: 3xN vector
  -- seed: 3 vector
  local ln = points:size(2)
  local p = points:clone()
  p:add(seed:repeatTensor(ln,1):t():clone():mul(-1))
  
  local s,v,d = torch.svd(p)
  
  local m,i = v:min(1)
  i = i[1]
  
  local eqn = torch.zeros(4)
  
  eqn:sub(1,3):copy(s:select(2,i):clone():contiguous())
  eqn:sub(1,3):div(eqn:norm(2,1):sum())
  eqn[4] = -eqn:sub(1,3):clone():cmul(seed):sum()
  
  return eqn
  
end

function refit_plane(xyz,pp,seed_h,seed_w, usenumpts)
  local nump = pp:double():sum()
  local points = torch.zeros(3,nump)
  
  for k=1,3 do
    points[k] = xyz[k][pp]:clone():contiguous()
  end
  
  local seed = xyz:select(2,seed_h):select(2,seed_w):clone():contiguous()
  
  if usenumpts and nump > usenumpts then
    
    local r = torch.rand(nump)
    local dst = (seed:repeatTensor(nump,1):t()-points):norm(2,1):squeeze()
    r = r:cmul(dst)
    
    r,ord = r:sort(true)
    points = points:index(2,ord:sub(1,usenumpts)):clone():contiguous()
    nump = usenumpts
  end
  
  local eqn = fit_plane_to_points_and_seed(points,seed,usenumpts)
  
  return eqn
end

function compute_seed_scores(xyz,mask,nmp,ndd,dist_err)

  local len = mask:eq(0):double():sum()
  
  local xyzr = torch.zeros(3,len)
  local nmpr = torch.zeros(3,len)
  
  local emask = mask:eq(0)
  
  for i=1,3 do
    xyzr[i] = xyz[i][emask]
    nmpr[i] = nmp[i][emask]
  end
  
  local nddr = ndd[emask]
  
  local seed_scores_r = torch.zeros(len)
  
  libpc.compute_seed_scores(torch.data(seed_scores_r), torch.data(xyzr:clone():contiguous()),
                                           torch.data(nmpr:clone():contiguous()),
                                           torch.data(nddr:clone():contiguous()),
                                           dist_err,len)
  
  local seed_scores = torch.zeros(xyz[1]:size())
  seed_scores[emask]=seed_scores_r
  seed_scores[mask] = 0
  
  return seed_scores:clone()
  
end

params = cmd:parse(process.argv)

_G.dist_err = 10
_G.norm_err = math.pi/3
_G.usenumpts = 40000
_G.maxiter = 25
_G.mindiff = 0.0001

_G.odfile = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,params.sweep_dir..'.od')
_G.imfile = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'plane_000_left.png')
_G.pc = ptcld.new(odfile)
_G.xyz = pc:get_xyz_map_no_mask()
_G.nmp,_G.ndd,_G.nph,_G.nth,_G.nmask = pc:get_normal_map()
_G.index,_G.rmask = pc:get_index_and_mask()

_G.eqn_table = {}

if util.fs.is_file(imfile) then
  _G.seed_scores = image.load(imfile)[1]:clone():contiguous()
else
  log.tic()
  _G.seed_scores = compute_seed_scores(xyz,nmask,nmp,ndd,dist_err)          
  print(log.toc())
  seed_scores = seed_scores:div(seed_scores:max()+0.0000001)
  image.save(imfile, seed_scores)
end

local j = 0

local covered = torch.zeros(pc.height,pc.width)
local covered_diff = 1
local covered_sum = covered:sum()

while (seed_scores:double():sum() > 0 and covered_diff > 0 and j < 50) do

  j = j+1
	local jj = ''..j
	while jj:len() < 3 do
		jj = '0'..jj
	end
	
	local best_imgf = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'plane_'..jj..'_best.png')
	local extr_imgf = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'plane_'..jj..'_extr.png')
	local covd_imgf = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'plane_'..jj..'_covd.png')
	local seed_imgf = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'plane_'..jj..'_left.png')
	
	print('  ',jj,seed_scores:double():sum())

	local seed_h,seed_w = get_best_seed(seed_scores)
	
	print(' ',seed_h,seed_w,seed_scores:max(), seed_scores[seed_h][seed_w])

	local eqn = get_seed_eqn(nmp,ndd,seed_h,seed_w)

	local pp = get_plane_points(xyz,nmp,eqn,dist_err,norm_err)

	local iter = 0
	local diff = 1.0
	local pps = pp:double():sum()
	local best_iter = iter
	local best_pps = pps
	local best_eqn = eqn
	local best_pp = pp:clone()

	print('  ','  ',iter, pps, diff, covered_sum)

	while (iter < maxiter and (diff >= mindiff) and pps > 0) do

		iter = iter + 1
	
		eqn = refit_plane(xyz,pp:cmul(covered:eq(0)),seed_h,seed_w,usenumpts)
		pp = get_plane_points(xyz,nmp,eqn,dist_err,norm_err)
	
		local pps = pp:double():sum()
	
		diff = (pps-best_pps)/pps
	
		if pps > best_pps then
			best_pps = pps
			best_iter = iter
			best_eqn = eqn
			best_pp = pp:clone()
		end
	
		print('  ','  ',iter, pps, diff)
	
	end

  local img = best_pp:double():repeatTensor(3,1,1)
  img:sub(1,1,math.max(1,seed_h-3),math.min(pc.height,seed_h+3),math.max(1,seed_w-3),math.min(pc.width,seed_w+3)):fill(1)
  img:sub(2,3,math.max(1,seed_h-3),math.min(pc.height,seed_h+3),math.max(1,seed_w-3),math.min(pc.width,seed_w+3)):fill(0)
	image.save(best_imgf, img)
	print('  ','  ','best', best_iter, best_pps)
	
	local extra = best_pp:cmul(covered:eq(0)):double()
	image.save(extr_imgf, extra)
	
	covered = covered+best_pp:double()
	covered = covered:gt(0):double()
	image.save(covd_imgf, covered)
	
	covered_diff = covered:sum()-covered_sum
	covered_sum = covered:sum()
	
	table.insert(eqn_table,{best_eqn,best_pps,extra:sum()})
	
	rmask[covered:byte()] = 1
	nmp,ndd,nph,nth,nmask = pc:get_normal_map(rmask)
	nmask[covered:byte()] = 1
	
	if util.fs.is_file(seed_imgf) then
	  seed_scores = image.load(seed_imgf)[1]:clone():contiguous()
	else
	  seed_scores = compute_seed_scores(xyz,nmask,nmp,ndd,dist_err)
	  seed_scores = seed_scores:div(seed_scores:max()+0.0000001)
	  image.save(seed_imgf,seed_scores)
	end
	
	collectgarbage()

end

local eqnf = path.join(params.arcs_dir, params.proj_dir,'work/planes',params.sweep_dir,'eqn.dat')
torch.save(eqnf,eqn_table)

if params.non_interactive then
	process.exit()
end