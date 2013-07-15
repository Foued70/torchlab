function image.load (filename, pixel_type, max_size, colorspace, dimensions)
   colorspace = colorspace or "RGB"
   dimensions = dimensions or "DHW"
   log.trace("Loading", filename,max_size,colorspace,dimensions)
   return image.Wand.new(filename,max_size):toTensor(pixel_type,colorspace,dimensions)
end

function image.save (filename,tensor,colorspace,dimensions)
   colorspace = colorspace or "RGB"
   dimensions = dimensions or "DHW"
   log.trace("Saving", filename,colorspace,dimensions)
   image.Wand.new(tensor,colorspace,dimensions):save(filename)
   return
end

function image.display(img)
   ui.ImgWidget.new():display(img)
end

----------------------------------------------------------------------
-- convolve(dst,src,ker,type)
-- convolve(dst,src,ker)
-- dst = convolve(src,ker,type)
-- dst = convolve(src,ker)
--
function image.convolve(...)
   local dst,src,kernel,mode
   local args = {...}
   if select('#',...) == 4 then
      dst = args[1]
      src = args[2]
      kernel = args[3]
      mode = args[4]
   elseif select('#',...) == 3 then
      if type(args[3]) == 'string' then
         src = args[1]
         kernel = args[2]
         mode = args[3]
      else
         dst = args[1]
         src = args[2]
         kernel = args[3]
      end
   elseif select('#',...) == 2 then      
      src = args[1]
      kernel = args[2]
   else
      print(dok.usage('convolve',
                       'convolves an input image with a kernel, returns the result', nil,
                       {type='torch.Tensor', help='input image', req=true},
                       {type='torch.Tensor', help='kernel', req=true},
                       {type='string', help='type: full | valid | same', default='valid'},
                       '',
                       {type='torch.Tensor', help='destination', req=true},
                       {type='torch.Tensor', help='input image', req=true},
                       {type='torch.Tensor', help='kernel', req=true},
                       {type='string', help='type: full | valid | same', default='valid'}))
      dok.error('incorrect arguments', 'convolve')
   end
   if mode and mode ~= 'valid' and mode ~= 'full' and mode ~= 'same' then
      dok.error('mode has to be one of: full | valid', 'convolve')
   end
   local md = (((mode == 'full') or (mode == 'same')) and 'F') or 'V'
   if kernel:nDimension() == 2 and src:nDimension() == 3 then
      local k3d = torch.Tensor(src:size(1), kernel:size(1), kernel:size(2))
      for i = 1,src:size(1) do
         k3d[i]:copy(kernel)
      end
      kernel = k3d
   end
   if dst then
      torch.conv2(dst,src,kernel,md)
   else
      dst = torch.conv2(src,kernel,md)
   end
   if mode == 'same' then
      local cx = dst:dim()
      local cy = cx-1
      local ofy = math.ceil(kernel:size(cy)/2)
      local ofx = math.ceil(kernel:size(cx)/2)
      dst = dst:narrow(cy, ofy, src:size(cy)):narrow(cx, ofx, src:size(cx))
   end
   return dst
end

----------------------------------------------------------------------
--- Returns a gaussian kernel.
--
function image.gaussian(...)
   -- process args
   local _, size, sigma, amplitude, normalize, 
   width, height, sigma_horz, sigma_vert = dok.unpack(
      {...},
      'gaussian',
      'returns a 2D gaussian kernel',
      {arg='size', type='number', help='kernel size (size x size)', default=3},
      {arg='sigma', type='number', help='sigma (horizontal and vertical)', default=0.25},
      {arg='amplitude', type='number', help='amplitute of the gaussian (max value)', default=1},
      {arg='normalize', type='number', help='normalize kernel (exc Amplitude)', default=false},
      {arg='width', type='number', help='kernel width', defaulta='size'},
      {arg='height', type='number', help='kernel height', defaulta='size'},
      {arg='sigma_horz', type='number', help='horizontal sigma', defaulta='sigma'},
      {arg='sigma_vert', type='number', help='vertical sigma', defaulta='sigma'}
   )
   
   -- local vars
   local center_x = width/2 + 0.5
   local center_y = height/2 + 0.5
   
   -- generate kernel
   local gauss = torch.Tensor(height, width)
   for i=1,height do
      for j=1,width do
         gauss[i][j] = amplitude * math.exp(-(math.pow((j-center_x)
                                                    /(sigma_horz*width),2)/2 
                                           + math.pow((i-center_y)
                                                   /(sigma_vert*height),2)/2))
      end
   end
   if normalize then
      gauss:div(gauss:sum())
   end
   return gauss
end

function image.gaussian1D(...)
   -- process args
   local _, size, sigma, amplitude, normalize
      = dok.unpack(
      {...},
      'gaussian1D',
      'returns a 1D gaussian kernel',
      {arg='size', type='number', help='size the kernel', default=3},
      {arg='sigma', type='number', help='Sigma', default=0.25},
      {arg='amplitude', type='number', help='Amplitute of the gaussian (max value)', default=1},
      {arg='normalize', type='number', help='Normalize kernel (exc Amplitude)', default=false}
   )

   -- local vars
   local center = size/2 + 0.5
   
   -- generate kernel
   local gauss = torch.Tensor(size)
   for i=1,size do
      gauss[i] = amplitude * math.exp(-(math.pow((i-center)
                                              /(sigma*size),2)/2))
   end
   if normalize then
      gauss:div(gauss:sum())
   end
   return gauss
end

----------------------------------------------------------------------
--- Returns a Laplacian kernel.
--
function image.laplacian(...)
   -- process args
   local _, size, sigma, amplitude, normalize, 
   width, height, sigma_horz, sigma_vert = dok.unpack(
      {...},
      'gaussian',
      'returns a 2D gaussian kernel',
      {arg='size', type='number', help='kernel size (size x size)', default=3},
      {arg='sigma', type='number', help='sigma (horizontal and vertical)', default=0.1},
      {arg='amplitude', type='number', help='amplitute of the gaussian (max value)', default=1},
      {arg='normalize', type='number', help='normalize kernel (exc Amplitude)', default=false},
      {arg='width', type='number', help='kernel width', defaulta='size'},
      {arg='height', type='number', help='kernel height', defaulta='size'},
      {arg='sigma_horz', type='number', help='horizontal sigma', defaulta='sigma'},
      {arg='sigma_vert', type='number', help='vertical sigma', defaulta='sigma'}
   )

   -- local vars
   local center_x = width/2 + 0.5
   local center_y = height/2 + 0.5
   
   -- generate kernel
   local logauss = torch.Tensor(height,width)
   for i=1,height do
      for j=1,width do
         local xsq = math.pow((i-center_x)/(sigma_horz*width),2)/2
         local ysq = math.pow((j-center_y)/(sigma_vert*height),2)/2
         local derivCoef = 1 - (xsq + ysq)
         logauss[i][j] = derivCoef * amplitude * math.exp(-(xsq + ysq))
      end
   end
   if normalize then
      logauss:div(logauss:sum())
   end
   return logauss
end


----------------------------------------------------------------------
--- Creates a random color mapping
--
function image.colormap(nbColor)
   -- note: the best way of obtaining optimally-spaced
   -- colors is to generate them around the HSV wheel,
   -- by varying the Hue component
   local map = torch.Tensor(nbColor,3)
   local huef = 0
   local satf = 0
   for i = 1,nbColor do
      -- HSL
      local hue = math.mod(huef,360)
      local sat = math.mod(satf,0.7) + 0.3
      local light = 0.5
      huef = huef + 39
      satf = satf + 1/9
      -- HSL -> RGB
      local c = (1 - math.abs(2*light-1))*sat
      local huep = hue/60
      local x = c*(1-math.abs(math.mod(huep,2)-1))
      local redp
      local greenp
      local bluep
      if huep < 1 then
         redp = c; greenp = x; bluep = 0
      elseif huep < 2 then
         redp = x; greenp = c; bluep = 0
      elseif huep < 3 then
         redp = 0; greenp = c; bluep = x
      elseif huep < 4 then
         redp = 0; greenp = x; bluep = c
      elseif huep < 5 then
         redp = x; greenp = 0; bluep = c
      else
         redp = c; greenp = 0; bluep = x
      end
      local m = light - c/2
      map[i][1] = redp + m
      map[i][2] = greenp + m
      map[i][3] = bluep + m
   end
   return map
end

----------------------------------------------------------------------
--- Creates a jet colour mapping - Inspired by http://www.metastine.com/?p=7
--
function image.jetColormap(nbColour)
   local map = torch.Tensor(nbColour,3)
   for i = 1,nbColour do
      local fourValue = 4 * i / nbColour
      map[i][1] = math.max(math.min(fourValue - 1.5, -fourValue + 4.5, 1),0)
      map[i][2] = math.max(math.min(fourValue -  .5, -fourValue + 3.5, 1),0)
      map[i][3] = math.max(math.min(fourValue +  .5, -fourValue + 2.5, 1),0)
   end
   return map
end



------------------------------------------------------------------------
--- Local contrast normalization of an image
--
-- do local contrast normalization on a given image tensor using kernel ker.
-- of kernel is not given, then a default 9x9 gaussian will be used
function image.lcn(im,ker)

   ker = ker or gaussian({size=9,sigma=1.591/9,normalize=true})
   local im = im:clone():type('torch.DoubleTensor')
   if not(im:dim() == 2 or (im:dim() == 3 and im:size(1) == 1)) then
     error('grayscale image expected')
   end
   if im:dim() == 3 then
      im = im[1]
   end
   mn = im:mean()
   sd = im:std()
   -- print(ker)

   -- 1. subtract the mean and divide by the standard deviation
   im:add(-mn)
   im:div(sd)

   -- 2. calculate local mean and std and normalize each pixel

   -- mean
   local lmn = torch.conv2(im, ker)
   -- variance
   local imsq = im:clone():cmul(im)
   local lmnsq = torch.conv2(imsq, ker)
   local lvar = lmn:clone():cmul(lmn)
   lvar:add(-1,lmnsq):mul(-1)
   -- avoid numerical errors
   lvar:apply(function(x) if x < 0 then return 0 end end)
   -- standard deviation
   local lstd  = lvar:sqrt()
   lstd:apply(function(x) if x < 1 then return 1 end end)

   -- apply normalization
   local shifti = math.floor(ker:size(1)/2)+1
   local shiftj = math.floor(ker:size(2)/2)+1
   --print(shifti,shiftj,lstd:size(),im:size())
   local dim = im:narrow(1,shifti,lstd:size(1)):narrow(2,shiftj,lstd:size(2)):clone()
   dim:add(-1,lmn)
   dim:cdiv(lstd)
   return dim:clone()

end
