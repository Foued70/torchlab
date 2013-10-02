libgm  = require '../image/libgm'
ffi    = require 'ffi'
ctorch = util.ctorch

Wand = Class()

-- TODO: add info function for reading exif information.
-- TODO: return call function to chain commands.

-- Constructor:
function Wand:__init(pathOrTensor, ...)
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
function Wand:load(path, width, height)
   -- Set canvas size:
   if width then
      -- This gives a cue to the wand that we don't need
      -- a large image than this. This is super cool, because
      -- it speeds up the loading of large images by a lot.
      libgm.MagickSetSize(self.wand, width, height or width)
   end

   -- Load image:
   status = libgm.MagickReadImage(self.wand, path)
   
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
function Wand:save(path, quality)
   -- Format?
   -- local format = (path:gfind('%.(...)$')() or path:gfind('%.(....)$')()):upper()
   -- if format == 'JPG' then format = 'JPEG' end
   -- self:format(format)

   -- Set quality:
   quality = quality or 85
   libgm.MagickSetCompressionQuality(self.wand, quality) 

   -- Save:
   status = libgm.MagickWriteImage(self.wand, path)
   
   -- Error?
   if status == 0 then
      error(self.name .. ': error saving image to path "' .. path .. '"')
   end

   -- return self
   return self
end

-- Size:
function Wand:size(width,height,filter)
   -- Set or get:
   if width or height then
      -- Get filter:
      filter = libgm[(filter or 'Cubic') .. 'Filter']

      -- Bounding box?
      if not height then
         -- in this case, the image must fit in a widthxwidth box:
         box = width
         cwidth,cheight = self:size()
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
         box = height
         cwidth,cheight = self:size()
         if cwidth < cheight then
            width = box
            height = box * cheight/cwidth
         else
            height = box
            width = box * cwidth/cheight
         end
      end

      -- Set dimensions:
      status = libgm.MagickResizeImage(self.wand, width, height, filter, 1.0)

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
function Wand:depth(depth)
   -- Set or get:
   if depth then
      -- Set depth:
      libgm.MagickSetImageDepth(self.wand, depth)

      -- return self
      return self
   else
      -- Get depth:
      return libgm.MagickGetImageDepth(self.wand) 
   end
end

-- Type:
function Wand:imagetype(type)
   -- Set or get:
   if type then
      -- Set type:
      libgm.MagickSetImageType(self.wand, imagetypes[type])

      -- return self
      return self
   else
      -- Get type:
      return imagetypes[tonumber(libgm.MagickGetImageType(self.wand))]
   end
end

-- Saved image Type:
function Wand:savedimagetype(type)
   -- Set or get:
   if type then
      type = imagetypes[type]
 
      -- Set type:
      libgm.MagickSetImageSavedType(self.wand, type)

      -- return self
      return self
   else
      -- Get type:
      return imagetypes[tonumber(libgm.MagickGetImageSavedType(self.wand))]
   end
end

-- Format:
function Wand:format(format)
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
colorspaces = {
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

for i = 0,#colorspaces do 
   colorspaces[colorspaces[i]] = i
end

-- Colorspaces:
function Wand:show_colorspaces()
   return colorspaces
end

imagetypes = {
   [0] = 'UndefinedType',
   'BilevelType',
   'GrayscaleType',
   'GrayscaleMatteType',
   'PaletteType',
   'PaletteMatteType',
   'TrueColorType',
   'TrueColorMatteType',
   'ColorSeparationType',
   'ColorSeparationMatteType',
   'OptimizeType'
}

for i = 0,#imagetypes do 
   imagetypes[imagetypes[i]] = i
end

function Wand:show_imagetypes()
   return imagetypes
end

-- Colorspace:
function Wand:colorspace(colorspace)
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
function Wand:flip()
   -- Flip image:
   libgm.MagickFlipImage(self.wand)

   -- return self
   return self
end

-- Flop:
function Wand:flop()
   -- Flop image:
   libgm.MagickFlopImage(self.wand)

   -- return self
   return self
end

-- Export to Blob:
function Wand:toBlob()
   -- Size pointer:
   local sizep = ffi.new('size_t[1]')

   -- To Blob:
   local blob = ffi.gc(libgm.MagickWriteImageBlob(self.wand, sizep), ffi.C.free)
   
   -- Return blob and size:
   return blob, tonumber(sizep[0])
end

-- Export to string:
function Wand:toString()
   -- To blob:
   local blob, size = self:toBlob()

   -- Lua string:
   local str = ffi.string(blob,size)

   -- Return string:
   return str
end

-- To Tensor:
function Wand:toTensor(dataType, colorspace, dims, nocopy)

   -- Dims:
   local width,height = self:size()

   -- Color space:
   colorspace = colorspace or 'RGB'  -- any combination of R, G, B, A, C, Y, M, K, and I
   -- common colorspaces are: RGB, RGBA, CYMK, and I

   -- Other colorspaces?
   if colorspace == 'HSL' or 
      colorspace == 'HWB' or 
      colorspace == 'LAB' or 
      colorspace == 'YUV' then
      -- Switch to colorspace:
      self:colorspace(colorspace)
      colorspace = 'RGB'
   end

   -- Type:
   dataType = dataType or 'byte'
   local tensorType, pixelType
   if dataType == 'byte' or 
      dataType == 'torch.ByteTensor' or 
      dataType == 'torch.CharTensor' then
         -- torch CharTensor is -127,127 
         tensorType = 'ByteTensor'
         pixelType = 'CharPixel'
   elseif dataType == 'float' or 
          dataType == 'torch.FloatTensor' then
      tensorType = 'FloatTensor'
      pixelType = 'FloatPixel'
   elseif dataType == 'double' or 
          dataType == 'torch.DoubleTensor' then
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
function Wand:fromBlob(blob,size)
   -- Read from blob:
   libgm.MagickReadImageBlob(self.wand, ffi.cast('const void *', blob), size)
   
   -- Save path:
   self.path = '<blob>'

   -- return self
   return self
end

-- Import from blob:
function Wand:fromString(string)
   -- Convert blob (lua string) to C string
   local size = #string
   blob = ffi.new('char['..size..']', string)

   -- Load blob:
   return self:fromBlob(blob, size)
end

-- From Tensor:
function Wand:fromTensor(tensor, colorspace, dims)

   -- Dims:
   local ndim = tensor:nDimension()
   local height,width,depth
   if ndim == 3 then 
      if not dims and tensor:size(1) <= 5 then
         dims = "DHW"
      end
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
   colorspace = colorspace
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

   -- TODO check if we need something similar to save grayscale or other.
   -- It seems that we need to set this before setting 
   if (#colorspace == 4) then 
      -- make sure we get the right type or alpha won't be saved
      self:imagetype("TrueColorMatteType")
   end

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

-- Description:
function Wand:info()
   -- Get information
   local str = ffi.gc(libgm.MagickDescribeImage(self.wand), ffi.C.free)
   return ffi.string(str)
end

-- Rotate
function Wand:rotate(angle_in_degrees,rgb)
   -- Create PixelWand:
   local background = 
      ffi.gc(libgm.NewPixelWand(), function(background)
                -- Collect:
                libgm.DestroyPixelWand(background)
                                   end)
   
   if rgb then 
      libgm.PixelSetRed(  background,rgb[1])
      libgm.PixelSetGreen(background,rgb[2])      
      libgm.PixelSetBlue( background,rgb[3])
   else
      libgm.PixelSetBlack(background,1)
   end
   libgm.MagickRotateImage(self.wand,background,angle_in_degrees)
end

-- equalize Histogram
function Wand:histogram_equalize()
   libgm.MagickEqualizeImage(self.wand)
end
