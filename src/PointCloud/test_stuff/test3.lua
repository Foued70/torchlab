local ufs = util.fs
local path = require 'path'
local fs = require 'fs'
local ptcld = PointCloud.PointCloud
local log = require '../util/log'
local io = require 'io'
local saliency = require '../image/saliency'


cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-arcs_dir','/Users/lihui815/Documents') --'/Users/lihui815/cloudlab/build/usr/local/tmp/arcs'
cmd:option('-proj_dir','motor-unicorn-0776')
cmd:option('-srce_dir', 'source/faro') --'source/po_scan/a'
cmd:option('-work_dir', 'work/planes')
cmd:option('-subd_dir', 'saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var')


cmd:option('-plane_bn', 'planes.t7')
cmd:option('-thrs_dir', 'thresholded')
cmd:option('-ex2d_dir', 'extents2d')
cmd:option('-ex3d_dir', 'extents3d')
cmd:option('-pthresh', 40)
cmd:option('-nthresh', math.pi/9)
cmd:option('-step', 2)

cmd:text()

-- parse input params
params = cmd:parse(process.argv)



local function make_directories(dir_table)
  for k,v in pairs(dir_table) do
    util.fs.mkdir_p(v)
  end
end

local function make_name_tables(params)
	
	local prjt_dir = path.join(params.arcs_dir,params.proj_dir)
	local srce_dir = path.join(prjt_dir,params.srce_dir)
	local work_dir = path.join(prjt_dir,params.work_dir)
	
  local planes_table = util.fs.dirs_only(work_dir)
  local fp_table = {}
  
  local numsweeps = #planes_table
  
  for i=1,numsweeps do
  
    local fp = planes_table[i]
    local sn = path.basename(fp)
    
    local xn = sn..'.xyz'
    local on = sn..'.od'
    local pn = sn..'.png'
    
    local tab ={}
    
    tab.threshold_dir = path.join(fp,params.thrs_dir)
    tab.extents2d_dir = path.join(fp,params.ex2d_dir)
    tab.extents3d_dir = path.join(fp,params.ex3d_dir)
    
    make_directories(tab)
    
    tab.plane_file = path.join(fp,params.subd_dir,params.plane_bn)
    tab.src_file = path.join(srce_dir,xn) --tab.src_file = path.join(srce_dir,sn,'sweep.xyz')
    tab.xyz_file = path.join(fp,xn)
    tab.od_file = path.join(fp,on)
    
    --[[
    local pc = ptcld.new(tab.src_file)
    pc:save_to_od(tab.od_file)
    pc:save_global_points_to_xyz(tab.xyz_file)
    --[[]]
    
    fp_table[i]=tab
  
  end
  
  collectgarbage()
  
  return fp_table
  
end

local function comp_npts_desc(a,b)
  return a.n_pts > b.n_pts
end

local function find_dists_to_plane(eqn,ptsT,mask,height,width)
  
  local distA = (eqn:repeatTensor(1,1) * ptsT):squeeze():clone()--:abs()
  local pdist = torch.zeros(height,width)
  pdist[mask:eq(0)] = distA:clone()
  
  local pnormal = eqn:sub(1,3):repeatTensor(height,width,1):transpose(2,3):transpose(1,2):clone()
  pnormal = pnormal:cdiv(pnormal:norm(2,1):repeatTensor(3,1,1))
  
  return pdist,pnormal
  
end

local function clean_up_border(border,height,width)

  local border_tmp = border:clone():gt(0)
  
  for h = 1,height-1 do
    local b1 = border_tmp[h]
    local b2 = border_tmp[h+1]
    for w=1,width-1 do
      
      local b11 = b1[w]
      local b12 = b1[w+1]
      local b21 = b2[w]
      local b22 = b2[w+1]
      if b11+b12+b21+b22 > 3 then
        border:sub(h,h+1,w,w+1):fill(0)
      end
    end
  end
  
  return border
  
end

local function find_extent_of_plane(emask,rmask,pdist,height,width,maxdp)
  
  local borderE = torch.zeros(height,width)
  local borderN = torch.zeros(height,width)
  local borderO = torch.zeros(height,width)
  
  for h=10+1,height-1 do
    
    local emask_ch = emask[h]
    
    if emask_ch:double():sum() > 0 then
    
      local emask_3h = emask:sub(h-1,h+1):t():clone():contiguous()
      local rmask_3h = rmask:sub(h-1,h+1):t():clone():contiguous()
      local pdist_3h = pdist:sub(h-1,h+1):t():clone():contiguous()
      
      for w=1+1,width-1 do
        
        local ec = emask_ch[w]
        if ec > 0 then
        
          local e = emask_3h:sub(w-1,w+1):clone():contiguous():resize(9):clone():contiguous()
          
          if e:sum() < 9 then
          
            local r = rmask_3h:sub(w-1,w+1):clone():contiguous():resize(9):clone():contiguous()
            local d = pdist_3h:sub(w-1,w+1):clone():contiguous():resize(9):clone():contiguous()
          
            local numE = 0
            local sumE = 0
            local numO = 0
						local sumO = 0
						local numN = 0
					
						local maxBlock = 0
					  dc = d[5]
					  
						for k=1,9 do
					
							if k ~= 5 then
						
								if e[k] == 0 then
							
									local rt = r[k]
									local dt = d[k] 
							
									if rt == 0 then
										numE = numE + 1
										numO = numO + 1
									
										if dt-dc >= 100 then
											sumE = sumE + 0.75+math.min(1,(dt-dc)/500)*0.25
										elseif dt-dc >= 50 then
											sumE = sumE + 0.50+math.min(1,(dt-dc)/100)*0.25
										elseif dt-dc >= 0 then
											sumE = sumE + 0.25+math.min(1,(dt-dc)/50)*0.25
											sumO = sumO + 0.75+math.min(1,(dt-dc)/50)*0.25
										elseif dt-dc >= -50 then
											sumO = sumO + 0.75+math.min(1,(dt-dc)/50)*0.25
										elseif dt-dc >= -100 then
											sumO = sumO + 0.50+math.min(1,(dt-dc)/100)*0.25
										else
											sumO = sumO + 0.25+math.min(1,(dt-dc)/500)*0.25
										end
								
									else
										numN = numN + 1
									end
							
								else
									if k < 3 then
										maxBlock = math.max(maxBlock,e[k] + e[k+1])
									elseif k > 7 then
										maxBlock = math.max(maxBlock,e[k] + e[k-1])
									elseif k % 3 == 0 then
										maxBlock = math.max(maxBlock,e[k] + e[k+3])
									elseif (k - 1) % 3 == 0 then
										maxBlock = math.max(maxBlock,e[k] + e[k-3])
									end
								end
							
							end
						end
					
						if maxBlock >= 2 then
							if numE > 0 then
								borderE[h][w] = sumE/numE
							end
							if numO > 0 then
								borderO[h][w] = sumO/numO
							end
							if numN > 0 then
								borderN[h][w] = 1
							end
						
						end
					
					end
          
        end
        
      end
    
    end
      
    collectgarbage()
  end
  
  borderE = clean_up_border(borderE,height,width)
  borderN = clean_up_border(borderN,height,width)
  borderO = clean_up_border(borderO,height,width)  
  
  collectgarbage()
  
  return borderE,borderN,borderO

end

local function find_threshold(pc,pdist,plane_normal,nmp,rmask,pthresh,nthresh,step)
  
  local numsteps = math.ceil(pthresh/step)
  
  local numer = plane_normal:clone():cmul(nmp):sum(1):squeeze()
	local denom = plane_normal:norm(2,1):cmul(nmp:norm(2,1)):squeeze()
	local min_angdf = numer:clone():cdiv(denom):acos()

	local best_nmp = nmp:clone()

	for n = 1,numsteps do

		local plane_thresh = n*step

		local pmask1 = pdist:le(plane_thresh)
		pmask1[rmask] = 0
		pmask3 = pmask1:repeatTensor(3,1,1)
		rmask1 = pmask1:eq(0)

		local plane_normal_masked = plane_normal:clone():cmul(pmask3:double())
		local nmp_masked,ndd_masked,p,t,nmask_masked = pc:get_normal_map(rmask1)
		numer = plane_normal_masked:clone():cmul(nmp_masked):sum(1):squeeze()
		denom = plane_normal_masked:norm(2,1):cmul(nmp_masked:norm(2,1)):squeeze()
		local angdf = numer:clone():cdiv(denom):acos()

		local improve_pts_1 = pmask1:clone():cmul(angdf:lt(min_angdf)):cmul(nmask_masked:eq(0))
		local improve_pts_3 = improve_pts_1:repeatTensor(3,1,1)

    if improve_pts_1:double():sum() > 0 then
  		min_angdf[improve_pts_1] = angdf[improve_pts_1]
	  	best_nmp[improve_pts_3] = nmp_masked[improve_pts_3]
	  end

		collectgarbage()

	end

	local nmp_diff = (nmp-best_nmp):norm(2,1):squeeze()

	local pmask1 = pdist:le(pthresh)
	pmask1[rmask] = 0
	pmask3 = pmask1:repeatTensor(3,1,1)
	rmask1 = pmask1:eq(0)
  
	local nmp_diff_mask = min_angdf:le(nthresh)
	local satisfy_mask = pmask1:clone():cmul(nmp_diff_mask)
	
	return satisfy_mask
	
end

local function clean_thresholded(img,height,width)

  local nb = torch.zeros(img:size())
  local bl = torch.zeros(img:size())
  
  bl:sub(2,height-1,2,width-1):fill(8)
  bl:sub(2,height,1,1):fill(5)
  bl:sub(2,height,width,width):fill(5)
  bl:sub(1,1,2,width-1):fill(5)
  bl:sub(height,height,2,width-1):fill(5)
  bl[1][1] = 3
  bl[1][width] = 3
  bl[height][1] = 3
  bl[height][width] = 3
  
  nb:sub(2,height):add(img:sub(1,height-1))
  nb:sub(1,height-1):add(img:sub(2,height))
  nb:sub(1,height,2,width):add(img:sub(1,height,1,width-1))
  nb:sub(1,height,1,width-1):add(img:sub(1,height,2,width))
  nb:sub(2,height,2,width):add(img:sub(1,height-1,1,width-1))
  nb:sub(1,height-1,1,width-1):add(img:sub(2,height,2,width))
  nb:sub(2,height,1,width-1):add(img:sub(1,height-1,2,width))
  nb:sub(1,height-1,2,width):add(img:sub(2,height,1,width-1))
  
  local nimg = img:clone()
  nimg[img:eq(1):cmul(nb:eq(0))] = 0
  nimg[img:eq(0):cmul(bl:eq(8)):cmul(nb:ge(7))] = 1
  nimg[img:eq(0):cmul(bl:eq(5)):cmul(nb:ge(4))] = 1
  nimg[img:eq(0):cmul(bl:eq(3)):cmul(nb:ge(3))] = 1
  
  return nimg
  
end

local function find_extents_of_sweep(i,fp_table,pthresh,nthresh,step)
  
  local fp = fp_table[i]
  local thrs_dir = fp.threshold_dir
  local ex2d_dir = fp.extents2d_dir
  local ex3d_dir = fp.extents3d_dir
  local od_file = fp.od_file
  local pl_file = fp.plane_file
  
  local pc = ptcld.new(od_file)
  
  local height = pc.height
  local width  = pc.width
  local count  = pc.count
  
  local index,rmask = pc:get_index_and_mask()
  local nmp,ndd,nph,nth,nmsk = pc:get_normal_map()
  local points = pc.points
  local ptsT = torch.ones(4,count)
  ptsT:sub(1,3):copy(points:t():clone():contiguous())

  local planes = torch.load(pl_file)
  table.sort(planes,comp_npts_desc)
  local planenum = #planes
  
  local border_cumul = torch.zeros(3,height,width)
  local prev_mask = torch.zeros(height,width)
  
  collectgarbage()
  
  log.tic()
  local toc = log.toc()
  
  for j=1,planenum do
  
    local p = planes[j]
    local eqn = p.eqn
    nthresh = p.norm_thres
    pthresh = p.res_thres * (2/p.score)
    print(pthresh,nthresh)
    
    local jj = ''..j
    
    while jj:len() < 3 do
      jj = '0'..jj
    end
    
    local fname = 'plane_'..jj
    local iname = fname..'.png'
    local xname = fname..'.xyz'
    
    local pdist,pnormal = find_dists_to_plane(eqn,ptsT,rmask,height,width)
    local emask_1 = find_threshold(pc,pdist:clone():abs(),pnormal,nmp,rmask,pthresh,nthresh,step):double()
    emask_1 = clean_thresholded(emask_1,height,width)
    
    --[[
    if prev_mask:double():sum() > 0 then
      emask_1[prev_mask:gt(0)] = 0
    end
    prev_mask[emask_1:gt(0)]=1
    --[[]]
    
    --[[]]
    local borderE,borderN,borderO = find_extent_of_plane(emask_1, rmask, pdist, height, width, maxdp)
    
    
    local border = torch.zeros(3,height,width)
    border[1] = borderE
    border[2] = borderN
    border[3] = borderO
    
    border_cumul = border_cumul:add(border)
    
    collectgarbage()
    image.save(path.join(ex2d_dir,iname),border)
    image.save(path.join(thrs_dir,iname),emask_1)
    if border:sum() > 0 then
      border:mul(255):floor()
      border[border:lt(0)] = 0
      border[border:gt(255)] = 255
      pc:save_mask_to_xyz(path.join(ex3d_dir,xname),border:sum(1):squeeze():gt(0),border)
    end
    --[[]]
    
    print(j, log.toc()-toc)
    toc = log.toc()
    
  end
  
  --[[]]
  if border_cumul:max() > 0 then
    border_cumul = border_cumul:div(border_cumul:max())
  end
  
  collectgarbage()  
   
  image.save(path.join(ex2d_dir,'all_planes.png'),border_cumul)
  if border_cumul:sum() > 0 then
    border_cumul:mul(255):floor()
    border_cumul[border_cumul:lt(0)] = 0
    border_cumul[border_cumul:gt(255)] = 255
    pc:save_mask_to_xyz(path.join(ex3d_dir,'all_planes.xyz'),border_cumul:sum(1):squeeze():gt(0),border_cumul)
  end
  --[[]]
  
end

local function find_extents_all_sweeps(fp_table,pthresh,nthresh,step,b,e)
  local numsweeps = #fp_table
  b = b or 1
  e = e or #fp_table
  b = math.max(b,1)
  e = math.min(e,#fp_table)
  for i=b,e do
    find_extents_of_sweep(i,fp_table,pthresh,nthresh,step)
    collectgarbage()
  end
  collectgarbage()
end

local function doAll(params,b,e)
  local fp_table = make_name_tables(params)
  find_extents_all_sweeps(fp_table,params.pthresh,params.nthresh,params.step,b,e)
  collectgarbage()
end

doAll(params,7,7)

