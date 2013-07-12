local libgm  = require '../image/libgm'
local ffi    = require 'ffi'
local ctorch = util.ctorch

local Image = Class()

-- Constructor:
function Image:__init(pathOrTensor, ...)
   -- Create new instance:
   self.name = 'magick.Image'
   self.path = '<>'
   self.buffers = {
      HWD = {},
      DHW = {}
   }

   -- Create Wand:
   self.wand = ffi.gc(libgm.NewMagickWand(), function(wand)
      -- Collect:
      libgm.DestroyMagickWand(wand)
   end)
  
   -- Arg?
   if type(pathOrTensor) == 'string' then
      -- Is a path:
      self:load(pathOrTensor, ...)
   
   elseif type(pathOrTensor) == 'userdata' then
      -- Is a tensor:
      self:fromTensor(pathOrTensor, ...)

   end
   
   -- 
   return image
end

-- Load image:
function Image:load(path, width, height)
   -- Set canvas size:
   if width then
      -- This gives a cue to the wand that we don't need
      -- a large image than this. This is super cool, because
      -- it speeds up the loading of large images by a lot.
      libgm.MagickSetSize(self.wand, width, height or width)
   end

   -- Load image:
   local status = libgm.MagickReadImage(self.wand, path)
   
   -- Error?
   if status == 0 then
      libgm.DestroyMagickWand(self.wand)
      error(self.name .. ': error loading image at path "' .. path .. '"')
   end

   -- Save path:
   self.path = path

   -- return self
   return self
end

-- Save image:
function Image:save(path, quality)
   -- Format?
   -- local format = (path:gfind('%.(...)$')() or path:gfind('%.(....)$')()):upper()
   -- if format == 'JPG' then format = 'JPEG' end
   -- self:format(format)

   -- Set quality:
   quality = quality or 85
   libgm.MagickSetCompressionQuality(self.wand, quality) 

   -- Save:
   local status = libgm.MagickWriteImage(self.wand, path)
   
   -- Error?
   if status == 0 then
      error(self.name .. ': error saving image to path "' .. path .. '"')
   end

   -- return self
   return self
end

-- Size:
function Image:size(width,height,filter)
   -- Set or get:
   if width or height then
      -- Get filter:
      local filter = libgm[(filter or 'Cubic') .. 'Filter']

      -- Bounding box?
      if not height then
         -- in this case, the image must fit in a widthxwidth box:
         local box = width
         local cwidth,cheight = self:size()
         if cwidth > cheight then
            width = box
            height = box * cheight/cwidth
         else
            height = box
            width = box * cwidth/cheight
         end
      end
      
      -- Min box?
      if not width then
         -- in this case, the image must cover a heightxheight box:
         local box = height
         local cwidth,cheight = self:size()
         if cwidth < cheight then
            width = box
            height = box * cheight/cwidth
         else
            height = box
            width = box * cwidth/cheight
         end
      end

      -- Set dimensions:
      local status = libgm.MagickResizeImage(self.wand, width, height, filter, 1.0)

      -- Error?
      if status == 0 then
         error(self.name .. ': error resizing image')
      end

      -- return self
      return self
   else
      -- Get dimensions:
      width,height = libgm.MagickGetImageWidth(self.wand), libgm.MagickGetImageHeight(self.wand)
   end
   --
   return width,height
end

-- Depth:
function Image:depth(depth)
   -- Set or get:
   if depth then
      -- Set depth:
      libgm.MagickSetImageDepth(self.wand, depth)

      -- return self
      return self
   else
      -- Get depth:
      local depth = libgm.MagickGetImageDepth(self.wand)
   end
   --
   return depth 
end

-- Format:
function Image:format(format)
   -- Set or get:
   if format then
      -- Set format:
      libgm.MagickSetImageFormat(self.wand, format)

      -- return self
      return self
   else
      -- Get format:
      format = ffi.string(libgm.MagickGetImageFormat(self.wand))
   end
   return format
end

-- Colorspaces available:
local colorspaces = {
   [0] = 'Undefined',
   'RGB',
   'GRAY',
   'Transparent',
   'OHTA',
   'XYZ',
   'YCC',
   'YIQ',
   'YPbPr',
   'YUV',
   'CMYK',
   'sRGB',
   'HSL',
   'HWB',
   'LAB',
   'CineonLogRGB',
   'Rec601Luma',
   'Rec601YCbCr',
   'Rec709Luma',
   'Rec709YCbCr'
}

-- Colorspaces:
function Image:colorspaces()
   return colorspaces
end

-- Colorspace:
function Image:colorspace(colorspace)
   -- Set or get:
   if colorspace then
      -- Set format:
      libgm.MagickSetImageColorspace(self.wand, libgm[colorspace .. 'Colorspace'])

      -- return self
      return self
   else
      -- Get format:
      colorspace = tonumber(ffi.cast('double', libgm.MagickGetImageColorspace(self.wand)))

      colorspace = colorspaces[colorspace]
   end
   return colorspace 
end

-- Flip:
function Image:flip()
   -- Flip image:
   libgm.MagickFlipImage(self.wand)

   -- return self
   return self
end

-- Flop:
function Image:flop()
   -- Flop image:
   libgm.MagickFlopImage(self.wand)

   -- return self
   return self
end

-- Export to Blob:
function Image:toBlob()
   -- Size pointer:
   local sizep = ffi.new('size_t[1]')

   -- To Blob:
   local blob = ffi.gc(libgm.MagickWriteImageBlob(self.wand, sizep), ffi.C.free)
   
   -- Return blob and size:
   return blob, tonumber(sizep[0])
end

-- Export to string:
function Image:toString()
   -- To blob:
   local blob, size = self:toBlob()

   -- Lua string:
   local str = ffi.string(blob,size)

   -- Return string:
   return str
end

-- To Tensor:
function Image:toTensor(dataType, colorspace, dims, nocopy)

   -- Dims:
   local width,height = self:size()

   -- Color space:
   colorspace = colorspace or 'RGB'  -- any combination of R, G, B, A, C, Y, M, K, and I
   -- common colorspaces are: RGB, RGBA, CYMK, and I

   -- Other colorspaces?
   if colorspace == 'HSL' or colorspace == 'HWB' or colorspace == 'LAB' or colorspace == 'YUV' then
      -- Switch to colorspace:
      self:colorspace(colorspace)
      colorspace = 'RGB'
   end

   -- Type:
   dataType = dataType or 'byte'
   local tensorType, pixelType
   if dataType == 'byte' or dataType == 'torch.ByteTensor' or dataType == 'torch.CharTensor' then
      tensorType = 'ByteTensor'
      pixelType = 'CharPixel'
   elseif dataType == 'float' or dataType == 'torch.FloatTensor' then
      tensorType = 'FloatTensor'
      pixelType = 'FloatPixel'
   elseif dataType == 'double' or dataType == 'torch.DoubleTensor' then
      tensorType = 'DoubleTensor'
      pixelType = 'DoublePixel'
   else
      error(self.name .. ': unknown data type ' .. dataType)
   end

   -- Dest:
   local tensor = self.buffers['HWD'][tensorType] or torch[tensorType]()
   tensor:resize(height,width,#colorspace)

   -- Cache tensor:
   self.buffers['HWD'][tensorType] = tensor

   -- Raw pointer:
   local ptx = torch.data(tensor)

   -- Export:
   libgm.MagickGetImagePixels(self.wand, 
                             0, 0, width, height,
                             colorspace, libgm[pixelType],
                             ffi.cast('unsigned char *',ptx))

   -- Dims:
   if dims == 'DHW' then
      -- Transposed Tensor:
      local tensorDHW = self.buffers['DHW'][tensorType] or tensor.new()
      tensorDHW:resize(#colorspace,height,width)

      -- Copy:
      tensorDHW:copy(tensor:transpose(1,3):transpose(2,3))

      -- Cache:
      self.buffers['DHW'][tensorType] = tensorDHW

      -- Return:
      tensor = tensorDHW
   end

   -- Return tensor
   if nocopy then
      return tensor
   else
      return tensor:clone()
   end
end

-- Import from blob:
function Image:fromBlob(blob,size)
   -- Read from blob:
   libgm.MagickReadImageBlob(self.wand, ffi.cast('const void *', blob), size)
   
   -- Save path:
   self.path = '<blob>'

   -- return self
   return self
end

-- Import from blob:
function Image:fromString(string)
   -- Convert blob (lua string) to C string
   local size = #string
   blob = ffi.new('char['..size..']', string)

   -- Load blob:
   return self:fromBlob(blob, size)
end

-- From Tensor:
function Image:fromTensor(tensor, colorspace, dims)

   -- Dims:
   local ndim = tensor:nDimension()
   local height,width,depth
   if ndim == 3 then 
      if dims == 'DHW' then
         depth,height,width= tensor:size(1),tensor:size(2),tensor:size(3)
         tensor = tensor:transpose(1,3):transpose(1,2)
      else -- dims == 'HWD'
         height,width,depth = tensor:size(1),tensor:size(2),tensor:size(3)
      end
   elseif ndim == 2 then 
      depth  = 1
      height = tensor:size(1)
      width  = tensor:size(2)
   end
   
   -- Force contiguous:
   tensor = tensor:contiguous()
   
   -- Color space:
   if not colorspace then
      if depth == 1 then
         colorspace = 'I'
      elseif depth == 3 then
         colorspace = 'RGB'
      elseif depth == 4 then
         colorspace = 'RGBA'
      else
      end
   end
   -- any combination of R, G, B, A, C, Y, M, K, and I
   -- common colorspaces are: RGB, RGBA, CYMK, and I

   -- Compat:
   assert(#colorspace == depth, self.name .. '.fromTensor: Tensor depth must match color space')

   -- Type:
   local ttype = torch.typename(tensor)
   local pixelType
   if ttype == 'torch.FloatTensor' then
      pixelType = 'FloatPixel'
   elseif ttype == 'torch.DoubleTensor' then
      pixelType = 'DoublePixel'
   elseif ttype == 'torch.ByteTensor' then
      pixelType = 'CharPixel'
   else
      error(self.name .. ': only dealing with float, double and byte')
   end
   
   -- Raw pointer:
   local ptx = torch.data(tensor)

   -- Resize image:
   self:load('xc:black')
   self:size(width,height)

   -- Export:
   libgm.MagickSetImagePixels(self.wand, 
                             0, 0, width, height,
                             colorspace, libgm[pixelType],
                             ffi.cast("unsigned char *", ptx))

   -- Save path:
   self.path = '<tensor>'

   -- return self
   return self
end

-- Show:
-- function Image:show(zoom)
--    -- Get Tensor from image:
--    local tensor = self:toTensor('float', nil,'DHW')
   
--    -- Display this tensor:
--    require 'image'
--    image.display({
--       image = tensor,
--       zoom = zoom
--    })

--    -- return self
--    return self
-- end

-- Description:
function Image:info()
   -- Get information
   local str = ffi.gc(libgm.MagickDescribeImage(self.wand), ffi.C.free)
   return ffi.string(str)
end


