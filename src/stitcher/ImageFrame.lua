path = require 'path'
saliency = require '../image/saliency'
blend = projection.util.blend
optimize = util.optimize.convex_binary_search
pi  = math.pi
pi2 = pi * 0.5

ImageFrame = Class()

function ImageFrame:__init(imgfile,hfov,vfov,scale,lambda,phi,psi)
  self.imgfile = imgfile
  
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


function ImageFrame:get_input_image()
  if force or (not self.input_image) then
    self.input_image = image.load(self.imgfile)
  end
  collectgarbage()
  return self.input_image:clone()
end

function ImageFrame:get_scaled_image()
  if (self.update_input or self.update_scale) or (not self.scaled_image) then
    local input_image = self:get_input_image()
    self.scaled_mask  = self:get_scale_mapper():get_mask():eq(0):double():repeatTensor(3,1,1):clone():contiguous()
    self.scaled_image = self:get_scale_mapper():remap(input_image):clone():contiguous()
    
    local wnd = image.Wand.new(self.scaled_image)
    
    local lab   = wnd:toTensor('double','LAB','DHW')
    self.scaled_salie = torch.zeros(self.scaled_image:size())
    
    self.scaled_salie[1] = saliency.high_entropy_features(lab[1]:clone())
    self.scaled_salie[1] = self.scaled_salie[1]:clone():pow(2):log1p()
    self.scaled_salie[1] = self.scaled_salie[1]:clone():div(self.scaled_salie[1]:max()+0.0000001)
    
    self.scaled_salie[2] = saliency.high_entropy_features(lab[2]:clone())
    self.scaled_salie[2] = self.scaled_salie[2]:clone():pow(2):log1p()
    self.scaled_salie[2] = self.scaled_salie[2]:clone():div(self.scaled_salie[2]:clone():max()+0.0000001)
    
    self.scaled_salie[3] = saliency.high_entropy_features(lab[3]:clone())
    self.scaled_salie[3] = self.scaled_salie[3]:clone():pow(2):log1p()
    self.scaled_salie[3] = self.scaled_salie[3]:clone():div(self.scaled_salie[3]:clone():max()+0.0000001)
    
    wnd = nil
    lab = nil
    
  end
  collectgarbage()
  return self.scaled_image:clone(), self.scaled_salie:clone(), self.scaled_mask:clone()
end

function ImageFrame:get_rotated_image()
  if (self.update_input or self.update_scale or self.update_rotat) or (not self.rotated_image) then
		local scaled_image, scaled_salie, scaled_mask = self:get_scaled_image()
	
		if self.psi ~= 0 then
	
			local deg = 180 * self.psi / pi
			local msk = scaled_mask:clone()
	
			local wnd_msk = image.wand.new(msk)
			wnd_msk:rotate(deg)
			self.rotated_mask  = wnd_msk:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd_msk = nil
	
			wnd_img = image.wand.new(scaled_image)
			wnd_img:rotate(deg)
			self.rotated_image = wnd_img:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd_img = nil
			
			wnd_img = image.wand.new(scaled_salie)
			wnd_img:rotate(deg)
			self.rotated_salie = wnd_img:toTensor('double', nil, 'DHW'):clone():contiguous()
		
			wnd_img = nil
			
		else
			self.rotated_image = scaled_image
			self.rotated_mask  = scaled_mask
			self.rotated_salie = scaled_salie
		end
	end
  
  collectgarbage()
  
  return self.rotated_image:clone(), self.rotated_salie:clone(), self.rotated_mask:clone()
end

function ImageFrame:get_output_image()
  if (self.update_input or self.update_scale or self.update_rotat or self.update_outpt) or (not self.output_image) then
    local rotated_image, rotated_salie, rotated_mask = self:get_rotated_image()
    self.output_image, self.output_mask  = self:get_output_mapper():remap_image_and_mask(rotated_image, rotated_mask)
    self.output_salie = self:get_output_mapper():remap(rotated_salie)
    local wnd = image.Wand.new(self.output_image)
    self.output_lab = wnd:toTensor('double','LAB','DHW')
    wnd = nil
  end
  collectgarbage()
  return self.output_image:clone(), self.output_salie:clone(), self.output_lab:clone(), self.output_mask:clone()
end

