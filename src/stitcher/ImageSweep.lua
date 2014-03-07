path = require 'path'
blend = projection.util.blend
log = require '../util/log'
pi  = math.pi
pi2 = pi * 0.5

ImageSweep = Class()


function ImageSweep:__init(img_files_tab,hfov,vfov,scale,phi,psi)

  self.img_files_tab = img_files_tab
  self.num_images    = #img_files_tab
  self.img_frams_tab = {}
  self.hfov_global   = hfov
  self.vfov_global   = vfov
  self.scale_global  = (scale or 1/5)
  self.phi_global    = (phi or 0)
  self.psi_global    = (psi or 0)
  self.phi_local = torch.zeros(self.num_images)
  self.psi_local = torch.zeros(self.num_images)
  self.lam_local = torch.zeros(self.num_images)
  
  local lambda = -pi
  
  for i = 1,self.num_images do
    print('loading '..self.img_files_tab[i])
    self.img_frams_tab[i] = stitcher.ImageFrame.new(img_files_tab[i],hfov,vfov,scale,lambda)
    self.lam_local[i] = lambda
    lambda = lambda + (2 * pi / self.num_images)
    collectgarbage()
  end
  
  collectgarbage()
  self:set_global_parameters(hfov,vfov,scale,phi,psi)
  collectgarbage()

end

function ImageSweep:clean()
  for i = 1,self.num_images do
    self.img_frams_tab[i]:clean()
  end
end

function ImageSweep:set_parameters_to_curr()
  local lml = self.lam_local:clone()
  local phl = self.phi_local:clone()
  local psl = self.psi_local:clone()
  self:set_global_parameters(self.hfov_global, self.vfov_global, self.scale_global, self.phi_global, self.psi_global)
  for i = 1,self.num_images do
    self:set_local_parameters(i, self.hfov_global, self.vfov_global, self.scale_global,
                                 lml[i], self.phi_global + phl[i], self.psi_global + psl[i])
  end
end

function ImageSweep:set_global_parameters(hfov,vfov,scale,phi,psi)
  self.hfov_global   = (hfov  or self.hfov_global)
  self.vfov_global   = (vfov  or self.vfov_global)
  self.scale_global  = (scale or self.scale_global)
  self.phi_global    = (phi   or self.phi_global)
  self.psi_global    = (psi   or self.psi_global)
  
  for i = 1,self.num_images do
    local tmpphi = phi
    local tmppsi = psi
    if phi then
      tmpphi = phi + self.phi_local[i]
    end
    if psi then
      tmppsi = psi + self.psi_local[i]
    end
    self:set_local_parameters(i,hfov,vfov,scale,nil,tmpphi,tmppsi)
    collectgarbage()
  end
  
  collectgarbage()
end

function ImageSweep:set_local_parameters(i,hfov,vfov,scale,lambda,phi,psi)
  self.lam_local[i] = (lambda or self.lam_local[i])
  if i == 1 then
    self.lam_local[i] = -pi
    lambda = -pi
  end
  if phi then
    self.phi_local[i] = phi - self.phi_global
  end
  if psi then
    self.psi_local[i] = psi - self.psi_global
  end
  self.img_frams_tab[i]:set_input_values(hfov,vfov,scale,lambda,phi,psi)
  collectgarbage()
end




function ImageSweep:optimize_local_all(psi_local_win,   phi_local_win,  lam_local_win, iter, rep, ran_size)
  
  local num_params  = self.num_images*3
  local psi_offset  = 0
  local phi_offset  = self.num_images
  local lam_offset  = self.num_images*2
  local winsizes    = torch.zeros(num_params)
  local curr_params = torch.zeros(num_params)
  
  winsizes[lam_offset+1]=0
  
  local function update_and_score_function(cp)
    self.psi_local  = cp:sub(psi_offset+1,phi_offset):clone()
    self.phi_local  = cp:sub(phi_offset+1,lam_offset):clone()
    self.lam_local  = cp:sub(lam_offset+1,num_params):clone()
    self:set_parameters_to_curr()
    local score = self:get_sift_score()
    collectgarbage()
    return score
  end
  
  collectgarbage()
  
  winsizes:sub(psi_offset+1,phi_offset):fill(psi_local_win)
  winsizes:sub(phi_offset+1,lam_offset):fill(phi_local_win)
  winsizes:sub(lam_offset+1,num_params):fill(lam_local_win)
  
  curr_params:sub(psi_offset+1,phi_offset):copy(self.psi_local)
  curr_params:sub(phi_offset+1,lam_offset):copy(self.phi_local)
  curr_params:sub(lam_offset+1,num_params):copy(self.lam_local) 
  
  collectgarbage()
  
  return self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)

end

function ImageSweep:optimize_global_all(psi_win, phi_win, iter, rep, ran_size, forward)

  local function update_and_score_function(params)
    self:set_global_parameters(nil,nil,nil, params[1],params[2])
    local score= self:get_sift_score()
    collectgarbage()
    return score
  end
  
  local curr_params = torch.Tensor({self.phi_global, self.psi_global})
  local winsizes    = torch.Tensor({        phi_win,         psi_win})
  
  collectgarbage()
  return self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
  
end

function ImageSweep:optimize_fov_all(hfov_win, vfov_win, iter, rep, ran_size, forward)

  local function update_and_score_function(params)
    self:set_global_parameters(params[1],params[2],nil, nil,nil)
    local score = self:get_sift_score()
    collectgarbage()
    return score
  end
  
  local curr_params = torch.Tensor({self.hfov_global, self.vfov_global})
  local winsizes    = torch.Tensor({        hfov_win,         vfov_win})
  
  collectgarbage()
  return self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
  
end

function ImageSweep:optimize_local_dir(i, psi_win, phi_win, lam_win, iter, rep, ran_size, forward)

  local fr = i
  local ov = i
  
  if forward then
    fr = i+1
    if fr > self.num_images then
      fr = self.num_images
    end
    ov = i
  else
    fr = (self.num_images - i) + 1
    ov = fr
  end
  
  print(i, fr, ov, forward)
  
  local function update_and_score_function(params)
    self:set_local_parameters(fr,nil,nil,nil, params[1], self.phi_global + params[2] ,self.psi_global + params[3])
    local score,weight = self:get_side_by_side_sift_score(ov)
    collectgarbage()
    return score
  end
  
  local curr_params = torch.Tensor({self.lam_local[fr], self.phi_local[fr], self.psi_local[fr]})
  local winsizes    = torch.Tensor({           lam_win,            phi_win,            psi_win})
  
  self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
  
  collectgarbage()
  
end

function ImageSweep:optimize_local_dir_all(psi_win, phi_win, lam_win, iter, rep, ran_size)

  -- keep copy of original
  local lam_tmp_0 = self.lam_local:clone()
  local phi_tmp_0 = self.phi_local:clone()
  local psi_tmp_0 = self.psi_local:clone()
  local score_0 = self:get_sift_score()
  print(0, score_0)
  collectgarbage()
  
  -- go forward
  for i = 1,self.num_images do
    self:optimize_local_dir(i, psi_win, phi_win, lam_win, iter, rep, ran_size, true)
    
    collectgarbage()
  end
  
  collectgarbage()
  
  local lam_tmp_1 = self.lam_local:clone()
  local phi_tmp_1 = self.phi_local:clone()
  local psi_tmp_1 = self.psi_local:clone()
  
  -- distribute error in first frame across
  local err_lam_1 = (lam_tmp_1[1] - lam_tmp_0[1])/self.num_images
  local err_phi_1 = (phi_tmp_1[1] - phi_tmp_0[1])/self.num_images
  local err_psi_1 = (psi_tmp_1[1] - psi_tmp_0[1])/self.num_images
  
  lam_tmp_1[1] = lam_tmp_0[1]
  phi_tmp_1[1] = phi_tmp_0[1]
  psi_tmp_1[1] = psi_tmp_0[1]
  
  lam_tmp_1:add(torch.range(0,self.num_images-1)*err_lam_1)
  phi_tmp_1:add(torch.range(0,self.num_images-1)*err_phi_1)
  psi_tmp_1:add(torch.range(0,self.num_images-1)*err_psi_1)
  
  for i = 1,self.num_images do
    self:set_local_parameters(i,nil,nil,nil,lam_tmp_1[i], self.phi_global + phi_tmp_1[i] , self.psi_global + psi_tmp_1[i])  
    collectgarbage()
  end
  
  local score_1 = self:get_sift_score()
  print(1, score_1)
  collectgarbage()
  
  -- reset
  for i = 1,self.num_images do
    self:set_local_parameters(i,nil,nil,nil,lam_tmp_0[i], self.phi_global + phi_tmp_0[i] , self.psi_global + psi_tmp_0[i])
    
    collectgarbage()
  end
  
  collectgarbage()
  
  -- go backward
  for i = 1,self.num_images-1 do
    self:optimize_local_dir(i, psi_win, phi_win, lam_win, iter, rep, ran_size, false)
  end
  
  collectgarbage()
  
  local lam_tmp_2 = self.lam_local:clone()
  local phi_tmp_2 = self.phi_local:clone()
  local psi_tmp_2 = self.psi_local:clone()
  
  -- distribute error in first frame across
  local err_lam_2 = (lam_tmp_2[1] - lam_tmp_0[1])/self.num_images
  local err_phi_2 = (phi_tmp_2[1] - phi_tmp_0[1])/self.num_images
  local err_psi_2 = (psi_tmp_2[1] - psi_tmp_0[1])/self.num_images
  
  lam_tmp_2:add((torch.ones(self.num_images):mul(self.num_images)-torch.range(0,self.num_images-1))*err_lam_2)
  phi_tmp_2:add((torch.ones(self.num_images):mul(self.num_images)-torch.range(0,self.num_images-1))*err_phi_2)
  psi_tmp_2:add((torch.ones(self.num_images):mul(self.num_images)-torch.range(0,self.num_images-1))*err_psi_2)
  
  lam_tmp_2[1] = lam_tmp_0[1]
  phi_tmp_2[1] = phi_tmp_0[1]
  psi_tmp_2[1] = psi_tmp_0[1]
  
  for i = 1,self.num_images do
    self:set_local_parameters(i,nil,nil,nil,lam_tmp_2[i], self.phi_global + phi_tmp_2[i] , self.psi_global + psi_tmp_2[i])  
    collectgarbage()
  end
  
  local score_2 = self:get_sift_score()
  print(2, score_2)
  collectgarbage()
  
  -- combine forward and backward
  local weight_forw = torch.range(0,self.num_images-1):mul(-1):add(self.num_images)
  local weight_back = torch.range(0,self.num_images-1)
  weight_forw[1] = 1
  weight_back[1] = 1
  local weight_totl = weight_forw + weight_back
  
  local lam_tmp_3  = (lam_tmp_1:clone():cmul(weight_forw) + lam_tmp_2:clone():cmul(weight_back)):cdiv(weight_totl)
  local phi_tmp_3  = (phi_tmp_1:clone():cmul(weight_forw) + phi_tmp_2:clone():cmul(weight_back)):cdiv(weight_totl)
  local psi_tmp_3  = (psi_tmp_1:clone():cmul(weight_forw) + psi_tmp_2:clone():cmul(weight_back)):cdiv(weight_totl)
  
  weight_forw = nil
  weight_back = nil
  weight_totl = nil
  collectgarbage()
  
  -- set the combined parameters
  for i = 1,self.num_images do
    self:set_local_parameters(i,nil,nil,nil,lam_tmp_3[i], self.phi_global + phi_tmp_3[i] , self.psi_global + psi_tmp_3[i])
    collectgarbage()
  end
  
  -- adjust first frame on phi and psi
  local function update_and_score_function(params)
    self:set_local_parameters(1,nil,nil,nil, nil, self.phi_global + params[1] ,self.psi_global + params[2])
    local score = self:get_sift_score()
    collectgarbage()
    return score
  end
  
  local curr_params = torch.Tensor({self.phi_local[1], self.psi_local[1]})
  local winsizes    = torch.Tensor({phi_win, psi_win})
  
  self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
  
  collectgarbage()
  
  -- get combined score
  local score_3 = self:get_sift_score()
  print(3, score_3)
  
  print('', score_0, score_3)
  
  -- compare original and current score, revert if necessary
  if score_0 <= score_3 then
    for i = 1,self.num_images do
      self:set_local_parameters(i,nil,nil,nil,lam_tmp_0[i], self.phi_global + phi_tmp_0[i] , self.psi_global + psi_tmp_0[i])
      
      collectgarbage()
    end
    collectgarbage()
  end
  
  -- print final score
  print(4, self:get_sift_score())
  
  collectgarbage()
  
end

function ImageSweep:optimize_global(update_function, curr_param, win_size, iter, rep, ran_size)
  
  local function update_and_score_function(params)
	  update_function(params[1])
	  collectgarbage()
	  return self:get_sift_score()
	end
	
	curr_param       = torch.Tensor({curr_param})
	local winsizes   = torch.Tensor({win_size})
	
	local best_params, best_score = self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
	
	collectgarbage()
	return best_params[1], best_score
	
end

function ImageSweep:optimize_local(update_function, curr_param, win_size, iter, rep, ran_size, first)
	
	local function update_and_score_function(params)
	  update_function(params)
	  collectgarbage()
	  return self:get_sift_score()
	end
	
	local winsizes   = torch.Tensor(curr_param:size()):fill(win_size)
	if not first then
	  winsizes[1] = 0
	end
	
	collectgarbage()
	
	return self:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
	
end

function ImageSweep:optimize(update_and_score_function, curr_params, winsizes, iter, rep, ran_size)
  local num_params  = winsizes:size(1)
  
  local best_params = curr_params
  local best_score  = update_and_score_function(best_params)
  
  local ran = math.pow(ran_size,math.min(self.num_images/2,num_params))
  
  for i = 1,iter do
    
    for j = 1,rep do
      
      for k = 1,ran do
        local rand   = (torch.rand(num_params)*2-1.0):cmul(winsizes)
        local params = curr_params + rand
        local score = update_and_score_function(params)
        
        if score < best_score then
          best_params = params
          best_score  = score
        end
        
        print('', i,j,k, score, best_score)
        collectgarbage()
        
      end
      
      curr_params = best_params:clone()
      collectgarbage()
      
    end
    
    winsizes = winsizes/ran_size
    collectgarbage()
  end
  
  print('', '', best_score)
  print(best_params:repeatTensor(1,1))
  update_and_score_function(best_params)
  collectgarbage()
  
  return best_params, best_score
  
end



function ImageSweep:get_side_by_side_score(i)
  
  local j = i+1
  if j > self.num_images then
    j = 1
  end
  
  local img_i, sal_i, lab_i, msk_i = self:get_frame_output(i)
  local img_j, sal_j, lab_j, msk_j = self:get_frame_output(j)
  
  local ovl_msk = msk_i:ge(1):cmul(msk_j:ge(1))
  
  if ovl_msk:double():sum() > 0 then
  
		local hght    = img_i:size(2)
		local wdth    = img_i:size(3)
		local coordh  = torch.range(1,hght):repeatTensor(wdth,1):t()
		local coordw  = torch.range(1,wdth):repeatTensor(hght,1)
	
		local coordh = coordh[ovl_msk[1]]
		local coordw = coordw[ovl_msk[1]]
	
		local minh = coordh:min()
		local maxh = coordh:max()
		local minw = coordw:min()
		local maxw = coordw:max()
	
		coordh = nil
		coordw = nil
		collectgarbage()
	
		img_i = img_i:sub(1,3,minh,maxh,minw,maxw):clone()
		img_j = img_j:sub(1,3,minh,maxh,minw,maxw):clone()
	
		sal_i = sal_i:sub(1,3,minh,maxh,minw,maxw):clone()
		sal_j = sal_j:sub(1,3,minh,maxh,minw,maxw):clone()
		
		lab_i = lab_i:sub(1,3,minh,maxh,minw,maxw):clone()
		lab_j = lab_j:sub(1,3,minh,maxh,minw,maxw):clone()
	
		msk_i = msk_i:sub(1,3,minh,maxh,minw,maxw):clone()
		msk_j = msk_j:sub(1,3,minh,maxh,minw,maxw):clone()
	
		ovl_msk = msk_i:ge(1):cmul(msk_j:ge(1)):double()
	
		local ovl_wgh = ovl_msk:double():sum()/3
	
		if ovl_wgh > 0 then
			local msk_l   = msk_i:clone():cmul(ovl_msk)
			local msk_r   = msk_j:clone():cmul(ovl_msk)
			local img_l   = img_i:clone():cmul(ovl_msk)
			local img_r   = img_j:clone():cmul(ovl_msk)
			local lab_l   = lab_i:clone():cmul(ovl_msk)
			local lab_r   = lab_j:clone():cmul(ovl_msk)
			local sal_l   = sal_i:clone():cmul(ovl_msk)
			local sal_r   = sal_j:clone():cmul(ovl_msk)
			
			local img_d = (img_l-img_r):pow(2):sum(1):squeeze()
			local lab_d = (lab_l-lab_r):pow(2):sum(1):squeeze()
			local sal_d = (img_l-img_r):pow(2):cmul(sal_l + sal_r):sum(1):squeeze() + (lab_l-lab_r):pow(2):cmul(sal_l + sal_r):sum(1):squeeze()
	
			local iscore = img_d:sum()/ovl_wgh
			local lscore = lab_d:sum()/ovl_wgh
			local sscore = sal_d:sum()/ovl_wgh
			local score  = iscore + lscore + sscore
	
	    collectgarbage()
	    
			return score,iscore,lscore,sscore,ovl_wgh
			
		end
		  
	end
		
	collectgarbage()
	return 1,1,1,1,0
  
end

function ImageSweep:get_score()
  local iscore = 0
  local lscore = 0
  local sscore = 0
  local weight = 0
  
  for i = 1,self.num_images do
    local score_i, iscore_i, lscore_i, sscore_i, ovl_wgh_i = self:get_side_by_side_score(i)
    iscore = iscore + iscore_i * ovl_wgh_i
    lscore = lscore + lscore_i * ovl_wgh_i
    sscore = sscore + sscore_i * ovl_wgh_i

    weight = weight + ovl_wgh_i
    collectgarbage()
  end
  
  collectgarbage()
  
  if weight > 0 then
    iscore = iscore/weight
    lscore = lscore/weight
    sscore = sscore/weight
  end
  
  local score = iscore + lscore + sscore
  
  collectgarbage()
  
  return score, iscore, lscore, sscore

end

function ImageSweep:get_panorama()

  local out_img, out_sal, out_lab, out_msk   = self:get_frame_output(1)
  local pan_img = out_img:clone()
  local pan_sal = out_sal:clone()
  local pan_lab = out_lab:clone()
  local pan_msk = out_msk:double()
  
  for i = 2,self.num_images do
  
    out_img, out_sal, out_lab, out_msk = self:get_frame_output(i)
    
    pan_img = pan_img + out_img
    pan_sal = pan_sal + out_sal
    pan_lab = pan_lab + out_lab
    pan_msk = pan_msk + out_msk:double()
    
    collectgarbage()
    
  end
  
  out_img = nil
  out_sal = nil
  out_lab = nil
  out_msk = nil
  
  collectgarbage()
  
  local vld    = pan_msk:gt(0)
  pan_img[vld] = pan_img[vld]:cdiv(pan_msk[vld])
  pan_sal[vld] = pan_sal[vld]:cdiv(pan_msk[vld])
  pan_lab[vld] = pan_lab[vld]:cdiv(pan_msk[vld])
  pan_msk      = pan_msk:eq(0)
  
  collectgarbage()
  
  return pan_img, pan_sal, pan_lab, pan_msk
end

function ImageSweep:get_blended_panorama()

  local rad_per_pix = self.img_frams_tab[1]:get_rad_per_pix()
  local out_img, out_sal, out_lab, out_msk   = self:get_frame_output(1)
  local hght = out_img:size(2)
  local wdth = out_img:size(3)
  
  coord    = torch.zeros(2,hght,wdth)
  coord[1] = torch.range(1,hght):repeatTensor(wdth,1):t()
  coord[2] = torch.range(1,wdth):repeatTensor(hght,1)
  
  local pp = torch.range(0,hght-1):repeatTensor(wdth,1):t()*rad_per_pix - pi/2
  local tt = torch.range(0,wdth-1):repeatTensor(hght,1)    *rad_per_pix - pi
  
  local weight = torch.ones(self.num_images,hght,wdth)
  
  for i=1,self.num_images do
    
    print('blend', i)
    
    local j = i+1
    if j > self.num_images then
      j = 1
    end
    
    local oii, osi, oli, omi = self:get_frame_output(i)
    local oij, osj, olj, omj = self:get_frame_output(j)
    local ovm           = (omi + omj):gt(1)
    ovm                 = ovm[1]:clone()
    
    local bdl           = torch.zeros(ovm:size())
    local bdr           = torch.zeros(ovm:size())
    
    local hmin = coord[1][ovm]:min()
    local hmax = coord[1][ovm]:max()
    
    weight[i][omi[1]:eq(0)]   = 0
    
    local crd = coord[2]:clone()
    
    for h = hmin,hmax do
      local c   = crd[h]:clone()[ovm[h]:clone()]:clone()
      local wmn = c:min()
      local wmx = c:max()
      local ch = h
      local cl = wmn
      local cr = wmx
      if ch > 0 and ch <= hght and cl > 0 and cl <= wdth and cr > 0 and cr <= wdth then
        bdl[ch][cl] = 1
        bdr[ch][cr] = 1
      else
        print(ch,cl,cr)
      end
    end
    
    bdl = bdl:byte()
    bdr = bdr:byte()
    
    crd = nil
    exl = nil
    exr = nil
    osi = nil
    osj = nil
    oli = nil
    olj = nil
    collectgarbage()
    
    local ppo = pp[ovm]:clone()
    local tto = tt[ovm]:clone()
    local ppl = pp[bdl]:clone()
    local ttl = tt[bdl]:clone()
    local ppr = pp[bdr]:clone()
    local ttr = tt[bdr]:clone()
    local nmo = ovm:double():sum()
    local nml = bdl:double():sum()
    local nmr = bdr:double():sum()
    
    local ttlc = ttl:clone():cos():repeatTensor(nmo,1)
    local ttls = ttl:clone():sin():repeatTensor(nmo,1)
    
    local ttoc = tto:clone():cos():repeatTensor(nml,1):t()
    local ttos = tto:clone():sin():repeatTensor(nml,1):t()
    
    local pplc = ppl:clone():repeatTensor(nmo,1)
    local ppoc = ppo:clone():repeatTensor(nml,1):t()
    
    local dl = (ttlc:clone():cmul(ttoc) + ttls:clone():cmul(ttos):cmul((pplc-ppoc):cos())):acos():min(2):squeeze():div(pi):clone():contiguous()
    
    ttlc = nil
    ttls = nil
    ttoc = nil
    ttos = nil
    pplc = nil
    ppoc = nil
    
    local ttrc = ttr:clone():cos():repeatTensor(nmo,1)
    local ttrs = ttr:clone():sin():repeatTensor(nmo,1)
    
    local ttoc = tto:clone():cos():repeatTensor(nmr,1):t()
    local ttos = tto:clone():sin():repeatTensor(nmr,1):t()
    
    local pprc = ppr:clone():repeatTensor(nmo,1)
    local ppoc = ppo:clone():repeatTensor(nmr,1):t()
    
    local dr = (ttrc:clone():cmul(ttoc) + ttrs:clone():cmul(ttos):cmul((pprc-ppoc):cos())):acos():min(2):squeeze():div(pi):clone():contiguous()
    
    ttrc = nil
    ttrs = nil
    ttoc = nil
    ttos = nil
    pprc = nil
    ppoc = nil
    
    collectgarbage()
    
    weight[i][ovm] = dr:pow(2)
    weight[j][ovm] = dl:pow(2)
    
    ppo = nil
    tto = nil
    ppl = nil
    ttl = nil
    ppr = nil
    ttr = nil
    dl  = nil
    dr  = nil
    
    bdl = nil
    bdr = nil
    
    collectgarbage()

  end
  
  weight_sum = weight:sum(1):repeatTensor(self.num_images,1,1)
  weight[weight_sum:gt(0)]=weight[weight_sum:gt(0)]:cdiv(weight_sum[weight_sum:gt(0)])
  print(weight_sum:max())
  weight_sum = weight_sum:div(weight_sum:max())
  
  collectgarbage()
  
  local pan_img = torch.zeros(3,hght,wdth)
  local pan_msk = torch.zeros(3,hght,wdth)
  
  for i = 1,self.num_images do
    local out_img, out_sal, out_lab, out_msk = self:get_frame_output(i)
    out_img = out_img:cmul(weight[i]:repeatTensor(3,1,1))
    out_msk = out_msk:cmul(weight[i]:repeatTensor(3,1,1))
    pan_img = pan_img + out_img
    pan_msk = pan_msk + out_msk
    
    out_sal = nil
    out_lab = nil
    collectgarbage()
  end
  
  pan_img = pan_img:cdiv(pan_msk:clone():add(0.0000001))
  pan_msk = pan_msk:eq(0)
  
  weight = nil
  weight_sum = nil
  
  collectgarbage()

  return pan_img, pan_msk
  
end

function ImageSweep:get_frame_output(i)
  local imfr = self.img_frams_tab[i]
  collectgarbage()
  return imfr:get_output_image()
end


function ImageSweep:get_seam_panorama()
  
  local img, sal, lab, msk = self:get_frame_output(1)
  local hght = img:size(2)
  local wdth = img:size(3)
  
  local pan_img = torch.zeros(3,hght,wdth)
  local pan_msk = torch.ones(3,hght,wdth)
  
  local seam_very_last = self:get_seam(self.num_images)
  local seam_last      = seam_very_last:clone()
  local seam_next      = self:get_seam(1)
  
  for i=1,self.num_images do
  
    if i > 1 then
      img, sal, lab, msk = self:get_frame_output(i)
      if i == self.num_images then
        seam_next = seam_very_last:clone()
      else
        seam_next = self:get_seam(i)
      end
    end
    
    sal = nil
    lab = nil
    collectgarbage()
  
    local curr_mask = msk[1]:ge(1)
    
    curr_mask[seam_last[1]:byte()] = 0
    curr_mask[seam_next[2]:byte()] = 0
    curr_mask = curr_mask:repeatTensor(3,1,1)
    
    pan_img[curr_mask] = img[curr_mask]
    pan_msk[curr_mask] = 0
    
    seam_last = seam_next:clone()
    
    collectgarbage()
    
  end
  
  collectgarbage()
  
  return pan_img, pan_msk
  
end


function ImageSweep:get_seam(i)

  local j = i + 1
  if j > self.num_images then
    j = 1
  end
  
  local img_i, sal_i, lab_i, msk_i = self:get_frame_output(i)
  local img_j, sal_j, lab_j, msk_j = self:get_frame_output(j)
  
  sal_i = nil
  sal_j = nil
  img_i = nil
  img_j = nil
  collectgarbage()
  
  local rad_per_pix = self.img_frams_tab[i]:get_rad_per_pix()
  local hght = lab_i:size(2)
  local wdth = lab_j:size(3)
  
  local coordh = torch.range(1,hght):repeatTensor(wdth,1):t()
  local coordw = torch.range(1,wdth):repeatTensor(hght,1)
  
  local ovl_msk_3 = msk_i:ge(1):cmul(msk_j:ge(1))
  local ovl_msk_1 = ovl_msk_3[1]:clone()
  
  local ovlh = coordh[ovl_msk_1]:clone()
  local ovlw = coordw[ovl_msk_1]:clone()
  
  local minh = ovlh:min()
  local maxh = ovlh:max()
  local minw = ovlw:min()
  local maxw = ovlw:max()
  local mxh  = maxh-minh+1
  local mxw  = maxw-minw+1
  
  local msk_i_sub       = msk_i:sub(1,3,minh,maxh,minw-1,maxw):clone()
  local msk_j_sub       = msk_j:sub(1,3,minh,maxh,minw,maxw+1):clone()
  local ovl_msk_1_sub   = (msk_i_sub:ge(1) + msk_j_sub:ge(1)):ge(1)[1]
  local lab_i_sub       = lab_i:sub(1,3,minh,maxh,minw-1,maxw):clone()
  local lab_j_sub       = lab_j:sub(1,3,minh,maxh,minw,maxw+1):clone()
  
  local distance      = (lab_i_sub:clone() - lab_j_sub:clone()):pow(2):sum(1):squeeze()
  
  local seam_score = torch.zeros(mxh,mxw+1)
  local prev_point = torch.zeros(mxh,mxw+1)
  local weight     = torch.zeros(mxh,mxw+1)
  local wtensor    = torch.range(1,mxw+1)
  
  local seam       = torch.zeros(mxh,mxw):repeatTensor(2,1,1)
  
  msk_i_sub = nil
  msk_j_sub = nil
  lab_i_sub = nil
  lab_j_sub = nil
  coordh    = nil
  coordw    = nil
  ovl_msk_3 = nil
  ovlh      = nil
  ovlw      = nil
  collectgarbage()
  
  for h = 1,mxh do
    
    local cwghts = ovl_msk_1_sub[h]:clone():double() --torch.ones(mxw+1)
    local cscore = cwghts:clone():cmul(distance[h]:clone())
    
    if h == 1 then
      weight:sub(h,h):copy(cwghts:clone())
      seam_score:sub(h,h):copy(cscore:clone())
      prev_point:sub(h,h):fill(0)
      collectgarbage()
      
    else
      local ph = math.max(1,h-1)
      local weight_ph       = weight[ph]:clone()
      local seam_score_ph   = seam_score[ph]:clone()
      
      local from_topp_scores = torch.zeros(3,mxw+1)      
      local from_topp_wghts = torch.zeros(3,mxw+1)
      
      from_topp_scores[2] = seam_score_ph + cscore
      from_topp_wghts[2]  = weight_ph + cwghts
      
      from_topp_scores:sub(1,1,2,mxw+1):copy(from_topp_scores:sub(2,2,1,mxw):clone())  -- from left
      from_topp_scores:sub(3,3,1,mxw):copy(from_topp_scores:sub(2,2,2,mxw+1):clone())  -- from right
      
      from_topp_wghts:sub(1,1,2,mxw+1):copy(from_topp_wghts:sub(2,2,1,mxw):clone())  -- from left
      from_topp_wghts:sub(3,3,1,mxw):copy(from_topp_wghts:sub(2,2,2,mxw+1):clone())  -- from right
      
      local from_topp_cumul = torch.Tensor(3,mxw+1):fill(math.huge)
      from_topp_cumul[from_topp_wghts:gt(0)] = from_topp_scores[from_topp_wghts:gt(0)]:clone():cdiv(from_topp_wghts[from_topp_wghts:gt(0)])
      
      local min_cumul, min_ind = from_topp_cumul:min(1)
      min_cumul = min_cumul:squeeze()
      min_ind   = min_ind:squeeze()
      if min_ind[1] == 1 then
        min_ind[1] = 2
      end
      if min_ind[mxw+1] == 3 then
        min_ind[mxw+1] = 2
      end
      
      for dim = 1,3 do
        local msk = min_ind:eq(dim)
        weight[h][msk] = from_topp_wghts[dim][msk]
        seam_score[h][msk] = from_topp_scores[dim][msk]
        collectgarbage()
      end
      
      prev_point[h] = min_ind:double() + wtensor - 2
      collectgarbage()
      
    end
    
    collectgarbage()
    
  end
  
  collectgarbage()
  
  local h = mxh
  
  local scores = seam_score[h]:clone():cdiv(weight[h]:clone())
  scores[weight[h]:eq(0)] = math.huge
  local minscore, minind = scores:min(1)
  minscore = minscore[1]
  minind   = minind  [1]
  
  local curind = minind
  
  distance = nil
  scores = nil
  seam_score = nil
  weight = nil
  minscore = nil
  minind = nil
  wtensor = nil
  collectgarbage()
  
  while h > 0 do
    
    if curind > 1 then  
      seam:sub(1,1,h,h,1,curind-1):fill(1)
    end
    if curind < mxw+1 then
      seam:sub(2,2,h,h,curind,mxw):fill(1)
    end
    
    curind = prev_point[h][curind]
    h = h-1
    
    collectgarbage()
    
  end
  
  collectgarbage()
  
  print('seam', i)
  
  local seam_lrg = torch.zeros(2,hght,wdth)
  seam_lrg:sub(1,1,minh,maxh,minw,maxw):copy(seam[1]:clone())
  seam_lrg:sub(2,2,minh,maxh,minw,maxw):copy(seam[2]:clone())
  
  seam = nil
  
  collectgarbage()
  
  return seam_lrg:clone()
  
end




function ImageSweep:get_side_by_side_sift_score(i)
  
  local j = i+1
  if j > self.num_images then
    j = 1
  end
  
  --local img_i, sal_i, lab_i, msk_i, fts_i = self:get_frame_output(i)
  --local img_j, sal_j, lab_j, msk_j, fts_j = self:get_frame_output(j)
  
  local img_i, sal_i, lab_i, msk_i = self:get_frame_output(i)
  local img_j, sal_j, lab_j, msk_j = self:get_frame_output(j)
  
  collectgarbage()
  
  local ovl_msk = msk_i:eq(1):cmul(msk_j:eq(1))[1]:clone()
  local ovl_wgh = ovl_msk:double():sum()
  
  if ovl_wgh > 0 then
    
    --[[
    local om128 = ovl_msk:double():repeatTensor(128,1,1)
    local om003 = ovl_msk:double():repeatTensor(3,1,1)
		fts_i = fts_i:cmul(om128)
		fts_j = fts_j:cmul(om128)
		img_i = img_i:cmul(om003)
		img_j = img_j:cmul(om003)
		sal_i = sal_i:cmul(om003)
		sal_j = sal_j:cmul(om003)
		lab_i = lab_i:cmul(om003)
		lab_j = lab_j:cmul(om003)
		
		
		local score = ((fts_i - fts_j):pow(2):sum() + 
		               (img_i - img_j):pow(2):sum() + 
		               (sal_i - sal_j):pow(2):sum() + 
		               (lab_i - lab_j):pow(2):sum())/ovl_wgh
		--[[]]
		
		local om003 = ovl_msk:double():repeatTensor(3,1,1)
		--fts_i = fts_i:cmul(om128)
		--fts_j = fts_j:cmul(om128)
		img_i = img_i:cmul(om003)
		img_j = img_j:cmul(om003)
		sal_i = sal_i:cmul(om003)
		sal_j = sal_j:cmul(om003)
		lab_i = lab_i:cmul(om003)
		lab_j = lab_j:cmul(om003)
		
		
		local score = ((lab_i:clone():cmul(sal_i) - lab_j:clone():cmul(sal_j)):pow(2):sum() + 
		               (img_i - img_j):pow(2):sum() + 
		               (sal_i - sal_j):pow(2):sum() + 
		               (lab_i - lab_j):pow(2):sum())/ovl_wgh
		
		
		
		img_i   = nil
	  img_j   = nil
    sal_i   = nil
    sal_j   = nil
    lab_i   = nil
    lab_j   = nil 
		fts_i   = nil
		fts_j   = nil
		ovl_msk = nil
		om128   = nil
		im003   = nil
		collectgarbage()
		
		return score, ovl_wgh
		  
	end
	
	img_i = nil
	img_j = nil
  sal_i = nil
  sal_j = nil
  lab_i = nil
  lab_j = nil
	fts_i   = nil
	fts_j   = nil
	ovl_msk = nil
	collectgarbage()
	return 1, 0
  
end

function ImageSweep:get_sift_score()

  local score  = 0
  local weight = 0
  
  for i = 1,self.num_images do
    local score_i, ovl_wgh_i = self:get_side_by_side_sift_score(i)

    score  = score  + score_i * ovl_wgh_i
    weight = weight + ovl_wgh_i
    
    collectgarbage()
  end
  
  collectgarbage()
  
  if weight > 0 then
    score = score/weight
  end
  
  collectgarbage()
  
  return score

end