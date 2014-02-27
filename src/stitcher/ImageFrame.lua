path = require 'path'
saliency = require '../image/saliency'
blend = projection.util.blend
optimize = util.optimize.convex_binary_search
extractor = opencv.Extractor
keypoints = opencv.KeyPoint
mat = opencv.Mat
pi  = math.pi
pi2 = pi * 0.5

ImageFrame = Class()

function ImageFrame:__init(imgfile,hfov,vfov,scale,lambda,phi,psi)
  self.imgfile          = imgfile
  self.imgdir           = path.dirname(imgfile)
  self.imgbase          = path.basename(imgfile, '.tiff')
  self.input_datfname   = path.join(self.imgdir, self.imgbase..'_input_img.dat')
  self.scaled_datfname  = path.join(self.imgdir, self.imgbase..'_scaled_img.dat')
  self.rotated_datfname = path.join(self.imgdir, self.imgbase..'_rotated_img.dat')
  self.output_datfname  = path.join(self.imgdir, self.imgbase..'_output_img.dat')
  
  self:clean()
  
  local img = self:get_input_image()
  self.wdth_i = (img:size(3) or 1)
  self.hght_i = (img:size(2) or 1)
  img = nil
  
  self.update_input = false
  self.update_scale = false
  self.update_rotat = false
  self.update_outpt = false
  
  self:set_input_values(hfov,vfov,scale,lambda,phi,psi)
  
end

function ImageFrame:set_input_values(hfov,vfov,scale,lambda,phi,psi)

  hfov   = hfov or (self.hfov_i or 0)
  vfov   = vfov or (self.vfov_i or 0)
  scale  = scale or (self.scale or 1/10)
  psi    = psi    or (self.psi    or 0)
  lambda = lambda or (self.lambda or 0)
  phi    = phi    or (self.phi    or 0)

  if ((hfov  ~= self.hfov_i) or 
      (vfov  ~= self.vfov_i) or 
      (scale ~= self.scale)) then
      
    self.update_input = true
    self.update_scale = true
    self.update_rotat = true
    self.update_outpt = true
    
    self.hfov_i = hfov
    self.vfov_i = vfov
    self.scale  = scale
  end
  
  if ((psi     ~= self.psi ) or
      (lambda  ~= self.lambda) or 
      (phi     ~= self.phi   )) then 
      
    self.update_rotat = true
    
    self.psi    = psi
    self.lambda = lambda
    self.phi    = phi
  end
  
end

function ImageFrame:get_rad_per_pix()
  self:update_input_values()
  return self.rad_per_pix
end


function ImageFrame:update_input_values()
  local h_rad_per_pix_i = self.hfov_i / self.wdth_i
  local v_rad_per_pix_i = self.vfov_i / self.hght_i
  self.rad_per_pix = math.min(h_rad_per_pix_i, v_rad_per_pix_i) / self.scale
end

function ImageFrame:update_scaled_values()
  
    self.hfov_s = self.hfov_i
    self.vfov_s = self.vfov_i
    self.wdth_s = self.hfov_s / self.rad_per_pix
    self.hght_s = self.vfov_s / self.rad_per_pix
  
end

function ImageFrame:update_rotated_values()

  if self.psi ~= 0 then
    local deg = 180 * self.psi / pi
    local msk = torch.ones(self.hght_s,self.wdth_s)
    local wnd = image.wand.new(msk)
    wnd:rotate(deg)
    msk = wnd:toTensor('double', nil, 'DHW'):eq(0):clone()
    wnd = nil
  
    self.wdth_r = msk:size(3)
    self.hght_r = msk:size(2)
    self.hfov_r = self.rad_per_pix * self.wdth_r
    self.vfov_r = self.rad_per_pix * self.hght_r
  else
    self.wdth_r = self.wdth_s
    self.hght_r = self.hght_s
    self.hfov_r = self.hfov_s
    self.vfov_r = self.vfov_s
  end
  
  collectgarbage()
  
end

function ImageFrame:update_output_values()

  self.hfov_o = 2 * pi
  self.vfov_o = 1 * pi
  self.wdth_o = self.hfov_o / self.rad_per_pix
  self.hght_o = self.vfov_o / self.rad_per_pix
  
end


function ImageFrame:get_input_projection()
  if self.update_input or (not self.input_projection) then
    self:update_input_values()
    self.input_projection = projection.GnomonicProjection.new(self.wdth_i, self.hght_i, 
                                                              self.hfov_i, self.vfov_i)
  end
  self.update_input = false
  return self.input_projection
end

function ImageFrame:get_scaled_projection()
  if self.update_scale or (not self.scaled_projection) then
    self:update_scaled_values()
    self.scaled_projection = projection.GnomonicProjection.new(self.wdth_s, self.hght_s, 
                                                               self.hfov_s, self.vfov_s)
  end
  self.update_scale = false
  return self.scaled_projection
end

function ImageFrame:get_rotated_projection()
  if self.update_rotat or (not self.rotated_projection) then
    self:update_rotated_values()
    self.rotated_projection = projection.GnomonicProjection.new(self.wdth_r, self.hght_r, 
                                                                self.hfov_r, self.vfov_r)
  end
  self.update_rotat = false
  return self.rotated_projection
end

function ImageFrame:get_output_projection()
  if self.update_outpt or (not self.output_projection) then
    self:update_output_values()
    self.output_projection = projection.SphericalProjection.new(self.wdth_o, self.hght_o, 
                                                                self.hfov_o, self.vfov_o)
  end
  self.update_outpt = false
  return self.output_projection
end


function ImageFrame:get_scale_mapper()
  if (self.update_input or self.update_scale) or (not self.scale_mapper) then
    self.scale_mapper = projection.Remap.new(self:get_input_projection(), self:get_scaled_projection())
  end
  return self.scale_mapper
end

function ImageFrame:get_output_mapper()
  if (self.update_rotat or self.update_outpt) or (not self.scale_mapper) then
    self.output_mapper = projection.Remap.new(self:get_rotated_projection(), self:get_output_projection())
    self.output_mapper:set_input_lambda_phi(self.lambda,self.phi)
  end
  return self.output_mapper
end


function get_sift_descriptor(use_img,use_msk)
  
  local d = use_img:dim()
  local h = use_img:size(d-1)
	local w = use_img:size(d)
	
	local crdw = torch.range(1,w):repeatTensor(h,1)[use_msk]
	local crdh = torch.range(1,h):repeatTensor(w,1):t()[use_msk]
	
	local minh = crdh:min()
	local maxh = crdh:max()
	
	use_msk:sub(minh, minh+4):fill(0)
	use_msk:sub(maxh-5, maxh):fill(0)
	for h = minh,maxh do
		local crdwh = crdw[crdh:eq(h)]:clone()
		local minw  = crdwh:min()
		local maxw  = crdwh:max()
		use_msk:sub(h, h, minw, minw+4):fill(0)
		use_msk:sub(h, h, maxw-5, maxw):fill(0)
		crdwh = nil
		collectgarbage()
	end
	
	local npts = use_msk:double():sum()
	
	crdh = nil
	crdw = nil
	collectgarbage()

	local k = torch.zeros(2,npts)
	k[1]    = torch.range(1,w):repeatTensor(h,1)[use_msk]:reshape(npts):clone():contiguous()
	k[2]    = torch.range(1,h):repeatTensor(w,1):t()[use_msk]:reshape(npts):clone():contiguous()
	k       = k:t()
	local kpts = keypoints.new(k:clone())
	local ext  = extractor.new('SIFT')

	k    = nil
	collectgarbage()

	local wnd = image.Wand.new(use_img)
	use_img   = wnd:toTensor('byte','G', 'HWD')
	m         = mat.new(use_img)
	local des = ext:compute(m, kpts.keypoints, kpts.npts)
	des       = des:toTensor():t():double():clone()
	local fts = torch.zeros(128,h,w)
	
	ext.extractor = nil
	ext           = nil
	wnd           = nil
	m.mat         = nil
	m.tensor      = nil
	m             = nil
	collectgarbage()
	
	for c = 1,128 do
		fts[c][use_msk] = des[c]:clone()
	end
	
	des           = nil
	collectgarbage()
	
	fts:div(255)
	
	use_img = nil
	use_msk = nil
	collectgarbage()
	
	return fts
	
end


function ImageFrame:get_input_image()
  return image.load(self.imgfile):clone():contiguous()
end

function ImageFrame:get_scaled_image()
  
  local fname = self.scaled_datfname
  if (self.update_input or self.update_scale) or (not util.fs.is_file(fname)) then
    local input_image = self:get_input_image()
    local scaled_mask  = self:get_scale_mapper():get_mask():eq(0):double():repeatTensor(3,1,1):clone():contiguous()
    local scaled_image = self:get_scale_mapper():remap(input_image):clone():contiguous()
    
    local wnd = image.Wand.new(scaled_image)
    
    local lab   = wnd:toTensor('double','LAB','DHW')
    local scaled_salie = torch.zeros(scaled_image:size())
    
    for i = 1,3 do
      scaled_salie[i] = saliency.high_entropy_features(lab[i]:clone())
      scaled_salie[i] = scaled_salie[i]:clone():pow(2):log1p()
      scaled_salie[i] = scaled_salie[i]:clone():div(scaled_salie[i]:max()+0.0000001)
    end
    
    wnd         = nil
    lab         = nil
    input_image = nil
    collectgarbage()
    
    torch.save(fname, {scaled_image, scaled_salie, scaled_mask})
    return scaled_image, scaled_salie, scaled_mask
  else
    local scaled_tab = torch.load(fname)
    local scaled_image = scaled_tab[1]:clone():contiguous()
    local scaled_salie = scaled_tab[2]:clone():contiguous()
    local scaled_mask  = scaled_tab[3]:clone():contiguous()
    
    scaled_tab[1] = nil
    scaled_tab[2] = nil
    scaled_tab[3] = nil
    scaled_tab    = nil
    collectgarbage()
    return scaled_image, scaled_salie, scaled_mask
  end
  
  
end

function ImageFrame:get_rotated_image()
  
  local fname = self.rotated_datfname
  if (self.update_input or self.update_scale or self.update_rotat) or (not util.fs.is_file(fname)) then
		local scaled_image, scaled_salie, scaled_mask = self:get_scaled_image()
		local rotated_image = scaled_image
		local rotated_mask  = scaled_mask
		local rotated_salie = scaled_salie
	
		if self.psi ~= 0 then
	
			local deg = 180 * self.psi / pi
	
			local wnd = image.wand.new(scaled_mask)
			wnd:rotate(deg)
			rotated_mask  = wnd:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd = nil
	
			wnd = image.wand.new(scaled_image)
			wnd:rotate(deg)
			rotated_image = wnd:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd = nil
			
			wnd = image.wand.new(scaled_salie)
			wnd:rotate(deg)
			rotated_salie = wnd:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd = nil
			
		end
		
		scaled_image = nil
		scaled_mask  = nil
		scaled_salie = nil
		collectgarbage()
		
		torch.save(fname, {rotated_image, rotated_salie, rotated_mask})
		return rotated_image, rotated_salie, rotated_mask
		
	else
	
	  local rotated_tab   = torch.load(fname)
	  local rotated_image = rotated_tab[1]:clone():contiguous()
	  local rotated_salie = rotated_tab[2]:clone():contiguous()
	  local rotated_mask  = rotated_tab[3]:clone():contiguous()
	  
	  rotated_tab[1] = nil
    rotated_tab[2] = nil
    rotated_tab[3] = nil
    rotated_tab    = nil
    collectgarbage()
    return rotated_image, rotated_salie, rotated_mask
    
  end

end

function ImageFrame:get_output_image()
  local fname = self.output_datfname
  if (self.update_input or self.update_scale or self.update_rotat or self.update_outpt) or (not util.fs.is_file(fname)) then
    local rotated_image, rotated_salie, rotated_mask = self:get_rotated_image()
    local output_image, output_mask                  = self:get_output_mapper():remap_image_and_mask(rotated_image, rotated_mask)
    local output_salie = self:get_output_mapper():remap(rotated_salie)
    local wnd          = image.Wand.new(output_image)
    local output_lab   = wnd:toTensor('double','LAB','DHW')
    --local output_fts   = get_sift_descriptor(output_lab[1]:clone(), output_mask[1]:eq(1))
    
    wnd = nil
    rotated_image = nil
    rotated_salie = nil
    rotated_mask  = nil
    collectgarbage()
    
    --torch.save(fname, {output_image, output_salie, output_lab, output_mask, output_fts})
    torch.save(fname, {output_image, output_salie, output_lab, output_mask})
    
    --return output_image, output_salie, output_lab, output_mask, output_fts
    return output_image, output_salie, output_lab, output_mask
  else
    
    local output_tab   = torch.load(fname)
    local output_image = output_tab[1]:clone():contiguous()
    local output_salie = output_tab[2]:clone():contiguous()
    local output_lab   = output_tab[3]:clone():contiguous()
    local output_mask  = output_tab[4]:clone():contiguous()
    --local output_fts   = output_tab[5]:clone():contiguous()
    
    output_tab[1] = nil
    output_tab[2] = nil
    output_tab[3] = nil
    output_tab[4] = nil
    --output_tab[5] = nil
    output_tab    = nil
    collectgarbage()
    
    --return output_image, output_salie, output_lab, output_mask, output_fts
    return output_image, output_salie, output_lab, output_mask
  end
end


function ImageFrame:clean()
  
  if util.fs.is_file(self.input_datfname) then
    os.execute(string.format('rm %s', self.input_datfname))
  end
  if util.fs.is_file(self.scaled_datfname) then
    os.execute(string.format('rm %s', self.scaled_datfname))
  end
  if util.fs.is_file(self.rotated_datfname) then
    os.execute(string.format('rm %s', self.rotated_datfname))
  end
  if util.fs.is_file(self.output_datfname) then
    os.execute(string.format('rm %s', self.output_datfname))
  end
  
end
