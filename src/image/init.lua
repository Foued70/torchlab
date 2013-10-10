require 'libimage'

-- local_wand reuses a single wand for these wrapper functions. This
-- fixes a bug when calling save multiple times in a row.  The
-- temporary wands created in the load and save helper functions would
-- end in a segfault. 
--
-- TODO: find source of bug when creating and destroying many wands in
-- a loop
--
-- WARNING: If saving mulitple images in a stream, you must call the
-- garbage collector (collectgarbage()) as the Wand will build up many
-- internal tensors, especially when passing DHW or non-contiguous tensors
local local_wand = image.Wand.new()

function image.load (filename, pixel_type, max_size, colorspace, dimensions)
   colorspace = colorspace or "RGB"
   dimensions = dimensions or "DHW"
   pixel_type = pixel_type or torch.getdefaulttensortype()
   log.info("Loading", filename,max_size,colorspace,dimensions)
   
   local_wand:load(filename,max_size)
   return local_wand:toTensor(pixel_type,colorspace,dimensions)
end

function image.save (filename,tensor,colorspace,dimensions)
   colorspace = colorspace or "RGB"
   dimensions = dimensions or "DHW"
   ndim = tensor:nDimension()
   if ndim == 2 then
      colorspace = "I"
      tensor = tensor:reshape(1,tensor:size(1),tensor:size(2))
   elseif ((dimensions == "DHW") and (tensor:size(1) == 4)) or ((dimensions == "HWD") and (tensor:size(3) == 4)) then
      colorspace = "RGBA"
   elseif ((dimensions == "DHW") and (tensor:size(1) == 1)) or ((dimensions == "HWD") and (tensor:size(3) == 1)) then
      colorspace = "I"
   end
   log.info("Saving", filename,colorspace,dimensions)
   local_wand:fromTensor(tensor,colorspace,dimensions)
   local_wand:save(filename)
   return
end


----------------------------------------------------------------------
-- warp
--
function image.warp(...)
   local dst,src,field
   local mode = 'bilinear'
   local offset_mode = true
   local args = {...}
   if select('#',...) == 5 then
      dst = args[1]
      src = args[2]
      field = args[3]
      mode = args[4]
      offset_mode = args[5]
   elseif select('#',...) == 4 then
      if type(args[3]) == 'string' then
         src = args[1]
         field = args[2]
         mode = args[3]
         offset_mode = args[4]
      else
         dst = args[1]
         src = args[2]
         field = args[3]
         mode = args[4]
      end
   elseif select('#',...) == 3 then
      if type(args[3]) == 'string' then
         src = args[1]
         field = args[2]
         mode = args[3]
      else
         dst = args[1]
         src = args[2]
         field = args[3]
      end
   elseif select('#',...) == 2 then
      src = args[1]
      field = args[2]
   else
      print(dok.usage('image.warp',
                      'warp an image, according to a flow field', nil,
                      {type='torch.Tensor', help='input image (KxHxW)', req=true},
                      {type='torch.Tensor', help='(y,x) flow field (2xHxW)', req=true},
                      {type='string', help='mode: bilinear | simple', default='bilinear'},
                      {type='string', help='offset mode (add (x,y) to flow field)', default=true},
                      '',
                      {type='torch.Tensor', help='destination', req=true},
                      {type='torch.Tensor', help='input image (KxHxW)', req=true},
                      {type='torch.Tensor', help='(y,x) flow field (2xHxW)', req=true},
                      {type='string', help='mode: bilinear | simple', default='bilinear'},
                      {type='string', help='offset mode (add (x,y) to flow field)', default=true}))
      dok.error('incorrect arguments', 'image.warp')
   end
   local dim2 = false
   if src:nDimension() == 2 then
      dim2 = true
      src = src:reshape(1,src:size(1),src:size(2))
   end
   dst = dst or src.new()
   dst:resize(src:size(1), field:size(2), field:size(3))
   src.image.warp(dst,src,field,((mode == 'bilinear') and true) or false, offset_mode)
   if dim2 then
      dst = dst[1]
   end
   return dst
end

----------------------------------------------------------------------
-- compresses an image between min and max
--
function image.minmax(args)
   local tensor    = args.tensor
   local min       = args.min
   local max       = args.max
   local symm      = args.symm or false
   local inplace   = args.inplace or false
   local saturate  = args.saturate or false
   local tensorOut = args.tensorOut or (inplace and tensor)
      or torch.Tensor(tensor:size()):copy(tensor)

   -- resize
   if args.tensorOut then
      tensorOut:resize(tensor:size()):copy(tensor)
   end

   -- saturate useless if min/max inferred
   if min == nil and max == nil then
      saturate = false
   end

   -- rescale min
   local fmin = 0
   if (min == nil) then
      if args.symm then
         fmin = math.max(math.abs(tensorOut:min()),math.abs(tensorOut:max()))
         min = -fmin
      else
         min = tensorOut:min()
      end
   end
   if (min ~= 0) then tensorOut:add(-min) end

   -- rescale for max
   if (max == nil) then
      if args.symm then
         max = fmin*2
      else
         max = tensorOut:max()
      end
   else
      max = max - min
   end
   if (max ~= 0) then tensorOut:div(max) end

   -- saturate
   if saturate then
      tensorOut.image.saturate(tensorOut)
   end

   -- and return
   return tensorOut
end

function image.combine(...)
   -- usage
   local _, input, padding, nrow, scaleeach, min, max, symm, saturate = dok.unpack(
      {...},
      'image.combine',
      'given a pack of tensors, returns a single tensor that contains a grid of all in the pack',
      {arg='input',type='torch.Tensor | table', help='input (HxW or KxHxW or Kx3xHxW or list)',req=true},
      {arg='padding', type='number', help='number of padding pixels between images', default=0},
      {arg='nrow',type='number',help='number of images per row', default=6},
      {arg='scaleeach', type='boolean', help='individual scaling for list of images', default=false},
      {arg='min', type='number', help='lower-bound for range'},
      {arg='max', type='number', help='upper-bound for range'},
      {arg='symmetric',type='boolean',help='if on, images will be displayed using a symmetric dynamic range, useful for drawing filters', default=false},
      {arg='saturate', type='boolean', help='saturate (useful when min/max are lower than actual min/max', default=true}
   )

   if type(input) == 'table' then
      -- pack images in single tensor
      local ndims = input[1]:dim()
      local channels = ((ndims == 2) and 1) or input[1]:size(1)
      local height = input[1]:size(ndims-1)
      local width = input[1]:size(ndims)
      local packed = torch.Tensor(#input,channels,height,width)
      for i,img in ipairs(input) do
         if scaleeach then
            packed[i] = image.minmax{tensor=input[i], min=min, max=max, symm=symm, saturate=saturate}
         else
            packed[i]:copy(input[i])
         end
      end
      return image.combine{input=packed,padding=padding,nrow=nrow,min=min,max=max,symmetric=symm,saturate=saturate}
   end

   if input:nDimension() == 4 and (input:size(2) == 3 or input:size(2) == 1) then
      -- arbitrary number of color images: lay them out on a grid
      local nmaps = input:size(1)
      local xmaps = math.min(nrow, nmaps)
      local ymaps = math.ceil(nmaps / xmaps)
      local height = input:size(3)+padding
      local width = input:size(4)+padding
      local grid = torch.Tensor(input:size(2), height*ymaps, width*xmaps):fill(input:max())
      local k = 1
      for y = 1,ymaps do
         for x = 1,xmaps do
            if k > nmaps then break end
            grid:narrow(2,(y-1)*height+1+padding/2,height-padding):narrow(3,(x-1)*width+1+padding/2,width-padding):copy(input[k])
            k = k + 1
         end
      end
      local mminput = image.minmax{tensor=grid, min=min, max=max, symm=symm, saturate=saturate}
      return mminput
   elseif input:nDimension() == 2  or (input:nDimension() == 3 and (input:size(1) == 1 or input:size(1) == 3)) then
      -- Rescale range
      local mminput = image.minmax{tensor=input, min=min, max=max, symm=symm, saturate=saturate}
      return mminput
   elseif input:nDimension() == 3 then
      -- arbitrary number of channels: lay them out on a grid
      local nmaps = input:size(1)
      local xmaps = math.min(nrow, nmaps)
      local ymaps = math.ceil(nmaps / xmaps)
      local height = input:size(2)+padding
      local width = input:size(3)+padding
      local grid = torch.Tensor(height*ymaps, width*xmaps):fill(input:max())
      local k = 1
      for y = 1,ymaps do
         for x = 1,xmaps do
            if k > nmaps then break end
            grid:narrow(1,(y-1)*height+1+padding/2,height-padding):narrow(2,(x-1)*width+1+padding/2,width-padding):copy(input[k])
            k = k + 1
         end
      end
      local mminput = image.minmax{tensor=grid, min=min, max=max, symm=symm, saturate=saturate}
      return mminput
   else
      xerror('input must be a HxW or KxHxW or Kx3xHxW tensor, or a list of tensors', 'image.combine')
   end
end

----------------------------------------------------------------------
-- super generic display function
--
function image.display(...)
   -- usage
   local _, input, min, max, scaleeach, padding, symm, nrow, saturate = dok.unpack(
      {...},
      'display',
      'displays a single image, with optional saturation/zoom',
      {arg='image', type='torch.Tensor | table', help='image (HxW or KxHxW or Kx3xHxW or list)', req=true},
      {arg='min', type='number', help='lower-bound for range'},
      {arg='max', type='number', help='upper-bound for range'},
      {arg='scaleeach', type='boolean', help='individual scaling for list of images', default=false},
      {arg='padding', type='number', help='number of padding pixels between images', default=0},
      {arg='symmetric',type='boolean',help='if on, images will be displayed using a symmetric dynamic range, useful for drawing filters', default=false},
      {arg='nrow',type='number',help='number of images per row', default=6},
      {arg='saturate', type='boolean', help='saturate (useful when min/max are lower than actual min/max', default=true}
   )

   input = image.combine{input=input, padding=padding, nrow=nrow, saturate=saturate,
                         scaleeach=scaleeach, min=min, max=max, symmetric=symm}
   -- if image is a table, then we treat if as a list of images
   -- if 2 dims or 3 dims and 1/3 channels, then we treat it as a single image
   if input:nDimension() == 2 then
      input = image.get3Dfrom1D(input)
   end
   if (input:nDimension() == 3 and (input:size(1) == 1 or input:size(1) == 3)) then
      ui.ImgWidget.new():display(input)
   else
      xerror('image must be a HxW or KxHxW or Kx3xHxW tensor, or a list of tensors', 'display')
   end

   return input

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

----- DISPLAY FUNCTIONS -------
function image.displayGrayInLua(tensor)
   image.display(image.get3Dfrom1D(tensor))
end

--cv_mat is grayscale image, points_mat is x,y location of points
--disp_color is defined as in types/Colors.lua
--displays square points for laziness
function image.displayPoints(img, points_mat, disp_color, radius)
   local color_tensor = image.get3Dfrom1DCopyChannels(img)
   for i=1, points_mat:size(1) do
      image.addPointTo3dImage(color_tensor, disp_color, points_mat[i][1], points_mat[i][2], radius)
   end
   image.display(color_tensor)
end

--takes in HW returns DHW
function image.get3Dfrom1D(tensor_th)
   local h = tensor_th:size(1)
   local w = tensor_th:size(2)
   -- no copying, or duplicating data just tricks with strides
   tensor_th = tensor_th:resize(1,h,w):expand(3,h,w)
   return tensor_th
end

--takes in HW returns DHW, but with channels copied (so if you want to change something in one channel, it doesnt get copied to all 3)
function image.get3Dfrom1DCopyChannels(tensor_th)
   local h = tensor_th:size(1)
   local w = tensor_th:size(2)
   -- no copying, or duplicating data just tricks with strides
   tensor_th = tensor_th:repeatTensor(3,1,1)
   return tensor_th
end

--helper function, changes the color of the squares of raidus centered at point_x,point_y to color_tensor
function image.addPointTo3dImage(color_tensor, disp_color, point_x, point_y, radius)
   for j=1,3 do
      color_tensor[{j,
      {math.max(point_x-radius,1), math.min(point_x+radius, color_tensor:size(2))},
      {math.max(point_y-radius,1), math.min(point_y+radius, color_tensor:size(3))}}]
      =disp_color[j]
   end
end


function image.warpAndCombineWithBorders(bestT, img_src, img_dest)
   local img_dest_copy = img_dest:clone()
   img_dest_copy:toTensor()[{{1,img_dest:size()[1]},{1}}]:fill(255)
   img_dest_copy:toTensor()[{{1,img_dest:size()[1]},{img_dest:size()[2]}}]:fill(255)
   img_dest_copy:toTensor()[{{1},{1,img_dest:size()[2]}}]:fill(255)
   img_dest_copy:toTensor()[{{img_dest:size()[1]}, {1,img_dest:size()[2]}}]:fill(255)

   local img_src_copy = img_src:clone()
   img_src_copy:toTensor()[{{1,img_src:size()[1]},{1}}]:fill(255)
   img_src_copy:toTensor()[{{1,img_src:size()[1]},{img_src:size()[2]}}]:fill(255)
   img_src_copy:toTensor()[{{1},{1,img_src:size()[2]}}]:fill(255)
   img_src_copy:toTensor()[{{img_src:size()[1]}, {1,img_src:size()[2]}}]:fill(255)

   return image.warpAndCombine(bestT, img_src_copy, img_dest_copy)
end

--transform img_src with bestT and combine with img_dest
--result is transformation should apply to img_src, translation should apply to img_dest, and combined image with
--transformed img_src on one channel and translated img_sest on another
function image.warpAndCombine(bestT, img_src, img_dest)
   Homography = geom.Homography
   local corners_src = image.getCorners(img_src)
   local corners_dest = image.getCorners(img_dest)
   local corners_src_warped = bestT:applyToPointsReturn2d(corners_src)

   local min_corner = torch.cat(corners_src_warped, corners_dest):min(2)
   local max_corner = torch.cat(corners_src_warped, corners_dest):max(2)

   --this is in lua coordinate system, swap x and y for opencv
   local size_x = (max_corner-min_corner)[1][1]
   local size_y = (max_corner-min_corner)[2][1]

   local translate =  Homography.new(0,torch.Tensor({-min_corner[1][1],-min_corner[2][1]}))
   local src_transformation_lua = translate:combineWith(bestT) --first apply bestT, then translate

   --switch to opencv coordinates!
   local src_transform = opencv.Mat.new(src_transformation_lua:getEquivalentCV())
   local dest_transform = opencv.Mat.new(translate:getEquivalentCV())   

   local warpedSrc  =  opencv.imgproc.warpImage(opencv.Mat.new(img_src), src_transform, size_y, size_x)
   local warpedDest =  opencv.imgproc.warpImage(opencv.Mat.new(img_dest), dest_transform, size_y, size_x)

   local tensor3d = torch.zeros(3,warpedSrc:size()[1], warpedSrc:size()[2])
   tensor3d[1] = warpedSrc:toTensor()
   tensor3d[2] = warpedDest:toTensor()
   return src_transform, dest_transform, tensor3d, warpedSrc, warpedDest
end

--returns the coordinates of the 4 corners
function image.getCorners(img_src)
   return torch.Tensor({{0,0}, {0, img_src:size()[2]}, {img_src:size()[1],0}, {img_src:size()[1],img_src:size()[2]}}):t()
end

--returns the coordinates of the 4 corners
function image.getCornersFromSize(size_x, size_y)
   return torch.Tensor({{0,0}, {0, size_y}, {size_x,0}, {size_x,size_y}}):t()
end
--only keep value greater than threshold
function image.thresholdReturnCoordinates(squareMatrix,threshold, le)
   local h=squareMatrix:size(1)
   local w=squareMatrix:size(2)
   local thresholded
   if(le) then
      thresholded = torch.lt(squareMatrix,threshold)
   else
      thresholded = torch.gt(squareMatrix,threshold)
   end
   x = torch.Tensor(h*w)
   i = 0
   x:apply(function() i = i + 1; return i end)
   local goodLocations = x[thresholded]
   local goodLocationsX = torch.ceil(goodLocations/w)
   local goodLocationsY = goodLocations:clone()
   goodLocationsY:apply(function(val) return (val -1)%w+1 end) 

   return goodLocationsX, goodLocationsY, squareMatrix[thresholded]
end

function image.median_filter(img,h,w)
   nchan = img:size(1)
   if nchan > 4 then 
      print("assuming single channel image")
      nchan = 1
      img:resize(util.util.add_slices(1,img:size()))
   end
   h = h or 3
   h2 = h/2
   w = w or 3
   w2 = w/2
   med = h*w/2

   imguf = img:unfold(2,h,1):unfold(3,w,1)
   ufh   = imguf:size(2)
   ufw   = imguf:size(3)
   imguf = imguf:reshape(nchan,ufh*ufw,h*w)
   s     = imguf:sort(3)
   s     = s[{{},{},med}]:reshape(nchan,ufh,ufw)

   img[{{},{h2,ufh+h2-1},{w2,ufw+w2-1}}]:copy(s)

end

return image
