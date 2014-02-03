local ufs = util.fs
local path = require 'path'
local fs = require 'fs'
local ptcld = PointCloud.PointCloud
local log = require '../util/log'
local io = require 'io'
local saliency = require '../image/saliency'
local imgraph = require '../imgraph'


cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('test')
cmd:text()
cmd:text('Options')
cmd:option('-arcs_dir','/Users/lihui815/Documents') --'/Users/lihui815/cloudlab/build/usr/local/tmp/arcs'

--[[
cmd:option('-scan_typ','pobot') --'pobot' --'faro'
cmd:option('-proj_dir','temporary-circle-6132') --'temporary-circle-6132' --'motor-unicorn-0776'
cmd:option('-srce_dir', 'source/po_scan/a') --'source/po_scan/a' --'source/faro'
cmd:option('-subd_dir', 'saliency_base_9_scale_1.8_n_scale_5_thres_60_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var')
--[[]]
cmd:option('-scan_typ','faro') --'pobot' --'faro'
cmd:option('-proj_dir','motor-unicorn-0776') --'temporary-circle-6132' --'motor-unicorn-0776'
cmd:option('-srce_dir', 'source/faro') --'source/po_scan/a' --'source/faro'
cmd:option('-subd_dir', 'saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var')
--[[]]

cmd:option('-work_dir', 'work/planes')
cmd:option('-plane_bn', 'planes.t7')
cmd:option('-thrs_dir', 'thresholded')
cmd:option('-ex2d_dir', 'extents2d')
cmd:option('-ex3d_dir', 'extents3d')
cmd:option('-segm_dir', 'segments')
cmd:option('-feat_dir', 'features')

cmd:option('-pthresh', 40)
cmd:option('-nthresh', math.pi/9)
cmd:option('-step', 2)

cmd:option('-mst_thrsh', 10)
cmd:option('-mst_minsz', 50)
cmd:option('-minhgt', 0.10)
cmd:option('-connex', 8)
cmd:option('-maxresidual',100)
cmd:option('-maxnormdiff',math.pi/3)

cmd:option('-numbestplanes',1)

cmd:option('-non_interactive',false)

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
    tab.segments_dir  = path.join(fp,params.segm_dir)
    tab.features_dir  = path.join(fp,params.feat_dir)
    
    make_directories(tab)
    
    tab.plane_file = path.join(fp,params.subd_dir,params.plane_bn)
    if params.scan_typ == 'faro' then
      tab.src_file = path.join(srce_dir,xn)
    elseif params.scan_typ == 'pobot' then
      tab.src_file = path.join(srce_dir,sn,'sweep.xyz')
    end
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
  local segm_dir = fp.segments_dir
  local feat_dir = fp.features_dir
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
  local xyz = pc:get_xyz_map_no_mask():clone()

  local planes = torch.load(pl_file)
  table.sort(planes,comp_npts_desc)
  local planenum = #planes
  
  --[[]]
  local border_cumul = torch.zeros(3,height,width)
  local prev_mask = torch.zeros(height,width)
  
  local planes_cumul = torch.zeros(3,height,width)
  local planes_sum = torch.zeros(3,height,width)
  
  local angstp1 = math.ceil(math.sqrt((planenum-2)/2))
  local angstp2 = math.ceil(math.sqrt((planenum-2)*2))
  
  local allp_cumul = torch.zeros(3*math.ceil(planenum/3),height,width)
  --[[]]
  
  collectgarbage()
  
  log.tic()
  local toc = log.toc()
  
  --[[]]
  local dmax = 0
  local amax = 0
  for j=1,planenum do
    if -planes[j].eqn[4] > dmax then
     dmax = -planes[j].eqn[4]
     amax = amax-planes[j].eqn[4]/planenum
    end
  end
  print(dmax,amax)
  
  for j=1,planenum do
  
    local p = planes[j]
    local eqn = p.eqn
    nthresh = p.norm_thres
    pthresh = p.res_thres * (2/p.score)
    
    --[[]]
    local pdist,pnormal = find_dists_to_plane(eqn,ptsT,rmask,height,width)
    
    local angdiff1 = pnormal:clone():cmul(nmp:clone()):sum(1)
    angdiff1 = angdiff1:cdiv(pnormal:norm(2,1):cmul(nmp:norm(2,1)):add(0.0000001))
    angdiff1 = angdiff1:squeeze():acos():abs()
    
    angdiff2 = angdiff1:clone()
    angdiff2:add(-math.pi/2):abs():mul(-1):add(math.pi/2)
    
    angdiff1 = angdiff1:mul(10):log1p():mul(-1)
    angdiff1:add(-angdiff1:min())
    angdiff1:div(angdiff1:max()+0.00000001)
    
    angdiff2 = angdiff2:mul(10):log1p():mul(-1)
    angdiff2:add(-angdiff2:min())
    angdiff2:div(angdiff2:max()+0.00000001)
    
    local img = pdist:clone():abs():log1p():mul(-1)
    img:add(-img:min())
    img:div(img:max()+0.000001)
    img:cmul(angdiff1):cmul(angdiff2)
    img[rmask]=0
    img[pdist:gt(params.maxresidual)] = 0
    img[angdiff1:gt(params.maxnormdiff)]=0
    --[[]]
    
    local jj = ''..j
    while jj:len() < 3 do
      jj = '0'..jj
    end
    
    allp_cumul[j]=img:div(img:max()+0.0000001)
    
    print(j, log.toc()-toc)
    toc = log.toc()
    print()
    
    collectgarbage()
    
  end
  
  --[[]]
  local allp_tmp = torch.zeros(allp_cumul:size())
  for j=1,math.min(params.numbestplanes,planenum) do
    local maxk = allp_cumul:max(1):squeeze()
    for k=1,planenum do
      local maxi = allp_cumul[k]:eq(maxk)
      allp_tmp[k][maxi] = allp_cumul[k][maxi]
      allp_cumul[k][maxi] = 0
    end
  end
  allp_cumul = allp_tmp:clone()
  allp_tmp = nil
  --[[]]
  
  
  for j=1,planenum do
  
    local p = planes[j]
    local eqn = p.eqn
    local img = allp_cumul[j]
    
    local i_ph
    local i_th
    
    if j == 1 then
      i_ph = -math.pi/2
      i_th = 0
    elseif j == 2 then
      i_ph = math.pi/2
      i_th = 0
    else
      local nummod = ((j-3) % angstp1) +1
      local numdiv = math.ceil((j-2)/angstp1)
      i_ph = (nummod)*(math.pi/(angstp1+1))-math.pi/2
      i_th = (numdiv)*(math.pi*2/angstp2)
      print(nummod,numdiv, i_th,i_ph)
    end
    
    local i_rgb = torch.Tensor(3)
    i_rgb[1] = math.cos(i_th)*math.cos(i_ph)
    i_rgb[2] = math.sin(i_th)*math.cos(i_ph)
    i_rgb[3] = math.sin(i_ph)
    i_rgb = (i_rgb+1)/2
    
    print(i_rgb[1],i_rgb[2],i_rgb[3])
    
    local scale = 1/(1+eqn[4]/dmax)
    scale = math.min(scale/dmax,1)
    --i_rgb = (((eqn:sub(1,3):clone()+1)/2))*scale
    
    local wght = img:repeatTensor(3,1,1)*p.score
    
    local img3 = wght:clone()
    for k=1,3 do
      if k == j%3+1 then
        img3[k] = img3[k]*(i_rgb[k])
      else
        img3[k] = img3[k]*(i_rgb[k])
      end
    end
    img3:div(img3:max()+0.000001)
    
    local jj = ''..j
    while jj:len() < 3 do
      jj = '0'..jj
    end
    local iname = 'plane_'..jj..'.png'
    
    image.save(path.join(feat_dir,iname),img3)
    
    planes_cumul = planes_cumul + img3
    planes_sum = planes_sum+wght
    
    print(j, log.toc()-toc)
    toc = log.toc()
    print()
    
    collectgarbage()
    
  end
  
  planes_cumul:cdiv(planes_sum+0.00000001)
  planes_cumul:div(planes_cumul:max()+0.00000001)
  image.save(path.join(segm_dir,'sg_pre.png'),planes_cumul)
  
  local mstg = torch.zeros(height,width)
  local wsg = torch.zeros(height,width)
  local mstc = torch.zeros(3,height,width)
  local wsc = torch.zeros(3,height,width)
  
  --[[]]
  for j=1,planenum do
    local jj = ''..j
    
    while jj:len() < 3 do
      jj = '0'..jj
    end
    
    local inputimg = allp_cumul[j]:clone()--allp_cumul:sub((j-1)*3+1,3*j):clone()
		local inputimgg = inputimg:clone()
	
		-- compute mst
		graph = imgraph.graph(inputimgg,params.connex)
		mstsegm = imgraph.segmentmst(graph, params.mst_thrsh*5, params.mst_minsz*5)
		mstsegmgraph = imgraph.graph(mstsegm,params.connex):sum(1):squeeze():gt(0):double()
		mstsegmcolor = imgraph.colorize(mstsegm)
		image.save(path.join(segm_dir, 'sg_3_ind_mst_gg_'..jj..'.png'),mstsegmgraph)
		image.save(path.join(segm_dir, 'sg_3_ind_mst_cc_'..jj..'.png'),mstsegmcolor)
		mstg = mstg+mstsegmgraph:gt(0):double()
		mstc = mstc+mstsegmcolor:double()
	
		-- compute the watershed of the graph
		
  	inputimgg = mstsegmcolor:clone()
		graph = imgraph.graph(inputimgg, params.connex)
		gradient = imgraph.graph2map(graph):gt(0):double()
		watershed = imgraph.watershed(gradient, params.minhgt*5, params.connex) 
		watershedgraph = imgraph.graph(watershed, params.connex) 
		watershedcc = imgraph.connectcomponents(watershedgraph, 0.5, true)
		watershed = watershed:eq(0):double()
		image.save(path.join(segm_dir, 'sg_3_ind_ws_gg_'..jj..'.png'),watershed)
		image.save(path.join(segm_dir, 'sg_3_ind_ws_cc_'..jj..'.png'),watershedcc)
		image.save(path.join(segm_dir, 'sg_3_ind_ws_gd_'..jj..'.png'),gradient)
		wsg = wsg+watershed:double()
		wsc = wsc+watershedcc:double()
		
		print(j, log.toc()-toc)
    toc = log.toc()
    print()
	
  end
  
  image.save(path.join(segm_dir, 'sg_3_sum_mst_gg.png'),mstg:gt(0):double())
  image.save(path.join(segm_dir, 'sg_3_sum_mst_cc.png'),mstc:clone():add(-mstc:min()):div((mstc:max()-mstc:min())+0.00000001))
  image.save(path.join(segm_dir, 'sg_3_sum_ws_gg.png'),wsg:gt(0):double())
  image.save(path.join(segm_dir, 'sg_3_sum_ws_cc.png'),wsc:clone():add(-wsc:min()):div((wsc:max()-wsc:min())+0.00000001))
	
	--[[]]
	
	inputimg = mstc:clone()
  inputimgg = inputimg
	
	-- compute mst
	graph = imgraph.graph(inputimgg,params.connex)
	mstsegm = imgraph.segmentmst(graph, params.mst_thrsh, params.mst_minsz)
	mstsegmgraph = imgraph.graph(mstsegm,params.connex):sum(1):squeeze():gt(0):double()
	mstsegmcolor = imgraph.colorize(mstsegm)
	image.save(path.join(segm_dir, 'sg_3_cbm_mst_gg.png'),mstsegmgraph)
	image.save(path.join(segm_dir, 'sg_3_cbm_mst_cc.png'),mstsegmcolor)
	
	-- compute the watershed of the graph
	inputimgg = mstsegmcolor:clone()
	graph = imgraph.graph(inputimgg, params.connex)
	gradient = imgraph.graph2map(graph):gt(0):double()
	watershed = imgraph.watershed(gradient, params.minhgt, params.connex) 
	watershedgraph = imgraph.graph(watershed, params.connex) 
	watershedcc = imgraph.connectcomponents(watershedgraph, 0.5, true)
	watershed = watershed:eq(0):double()
	
	image.save(path.join(segm_dir, 'sg_3_cbm_ws_gg.png'),watershed)
	image.save(path.join(segm_dir, 'sg_3_cbm_ws_cc.png'),watershedcc)
	image.save(path.join(segm_dir, 'sg_3_cbm_ws_gd.png'),gradient)
	
	--[[]]
	
	inputimg = planes_cumul:clone()
  inputimgg = inputimg
	
	-- compute mst
	graph = imgraph.graph(inputimgg,params.connex)
	mstsegm = imgraph.segmentmst(graph, params.mst_thrsh, params.mst_minsz)
	mstsegmgraph = imgraph.graph(mstsegm,params.connex):sum(1):squeeze():gt(0):double()
	mstsegmcolor = imgraph.colorize(mstsegm)
	image.save(path.join(segm_dir, 'sg_3_cbp_mst_gg.png'),mstsegmgraph)
	image.save(path.join(segm_dir, 'sg_3_cbp_mst_cc.png'),mstsegmcolor)
	
	-- compute the watershed of the graph
	inputimgg = mstsegmcolor:clone()
	graph = imgraph.graph(inputimgg, params.connex)
	gradient = imgraph.graph2map(graph):gt(0):double()
	watershed = imgraph.watershed(gradient, params.minhgt, params.connex) 
	watershedgraph = imgraph.graph(watershed, params.connex) 
	watershedcc = imgraph.connectcomponents(watershedgraph, 0.5, true)
	watershed = watershed:eq(0):double()
	
	image.save(path.join(segm_dir, 'sg_3_cbp_ws_gg.png'),watershed)
	image.save(path.join(segm_dir, 'sg_3_cbp_ws_cc.png'),watershedcc)
	image.save(path.join(segm_dir, 'sg_3_cbp_ws_gd.png'),gradient)
	
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

doAll(params,1,1)

if params.non_interactive then
  process.exit()
end

