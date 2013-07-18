Class()
-- These functions are about addressing (addr) data in tensor by
-- memory location. Other names for this file could be "loc" or
-- "index" or "offset" -- all of which are more confusing. "addr" has
-- the added conotation that we are talking about locations in memory
-- and not in space.

-- An offset is a torch.LongTensor containing values to lookup
-- locations in another tensor's memory as though you were looking in
-- that tensor's underlying data in 1D as it is aligned in memory. For
-- example, we often need to keep track of x and y locations in a
-- pixel map as well as the offset lookup values to quickly access the
-- data at that pixel location. These functions help to move back and
-- forth between the offsets and coordinates (offset_to_pixel_coords,
-- pixel_coords_to_offset) and to quickly access the data based on an offset
-- Tensor (remap).  The offset tensor itself can have any number of
-- dimensions which shape the result of the remapping.

-- A coord (or coordinate) is an address in multiple parts (an x and y
-- coordinate or a height (h) and width (w) for 2D).  pixel_coords are
-- nCoords x nSamples because we often have to operate on coordinate
-- dimensions independently.  Storing pixel_coords nCoords by nSamples keeps
-- each dimension contiguous in memory speeding up operations such as
-- x = sin(x) + pi * cos(y) etc.  Other types of coordinates are 3D
-- x,y,z though we don't often store those densely in a tensor and a
-- CubeMap 6 x X x Y.

-- using <offset> and (optional) <mask> remap <img> to
-- <out_image>.  <out_image> gets it's dimensions from <offset>
function remap(img, offset, mask, out_image)
   out_image          = out_image or torch.Tensor():typeAs(img)
   local index_n_elem = offset:nElement()
   local index_size   = offset:size()
   local index_dim    = offset:nDimension()

   local image_dim    = img:nDimension()

   local n_slices     = img:size(1)

   -- allow for single slice images with no first dimension
   if (image_dim == index_dim) then
      img = img:reshape(util.util.add_slices(1,img:size()))
      n_slices = 1 
   end

   -- make output 3 x H x W (or n_slices by whatever)
   local out_size = util.util.add_slices(n_slices, index_size)
   local input_elements_per_slice = img[1]:nElement()

   -- eg. n channel (RGB) (n x (h * w))
   out_image:resize(n_slices,index_n_elem)
  
   -- the low level torch code only allows for 1D indexing.
   -- reshape only changes the size locally
   if (index_dim ~= 1) then
      offset = offset:reshape(index_n_elem)
      if mask then 
         if (mask:nElement() ~= index_n_elem) then
            error("mask not same size as offset")
         else
            mask = mask:reshape(index_n_elem)
         end
      end
   end
   
   for d = 1,n_slices do -- loop through channels one at a time
      local imgd = img[d]

      -- flatten to 1D for fast indexing
      imgd:resize(input_elements_per_slice)

      out_image[d]    = imgd[offset]
      if mask then 
         out_image[d][mask] = 0 -- erase out of bounds values
      end
   end
   
   out_image:resize(out_size)
   
   return out_image
end

-- make mask for out of bounds values of coordinates defined in <max_dims>,<min_dims>
function mask_out_of_bounds(pixel_coords,max_dims,min_dims,mask)
   local nCoords = pixel_coords:size(1)
   if max_dims:size(1) ~= nCoords then
      printf("nCoords: %d max_dims: %d",nCoords,max_dims:size(1))
      error("not same number of dimensions as max_dims")
   end
   if not min_dims then
      min_dims = torch.ones(nCoords)
   end

   mask = mask or torch.ByteTensor(pixel_coords[1]:size())
   mask:resize(pixel_coords[1]:size()):zero()

   -- out of bounds check
   for d = 1,nCoords do 
      mask = mask + pixel_coords[d]:ge(min_dims[d]) + pixel_coords[d]:le(max_dims[d]) 
   end
   -- reset mask to 0 and 1 (valid parts of mask must pass 2 tests for
   -- each dimenstion (less than and greater than) We need the mask to
   -- reset bad pixels so the 1s are the out of bound pixels we want
   -- to replace
   local valid = nCoords * 2
   mask[mask:ne(valid)] = 1  -- out of bounds
   mask[mask:eq(valid)] = 0  -- in bounds
   return mask
end

-- convert the x and y index into a single 1D offset (y * stride + x)
function pixel_coords_to_offset(pixel_coords,stride,mask,offset)

   offset = offset or torch.LongTensor(pixel_coords[1]:size())
   offset:resize(pixel_coords[1]:size())

   -- CAREFUL strides are counted in the original raw image, not in the
   -- projection which can be scaled if xmap:size() is not equal to
   -- the size original image in which we index this function will
   -- return all the wrong values.
   stride = stride or error("must pass stride")

   -- CAREFUL must floor before multiplying by stride or does not make
   -- sense.  -0.5 then floor is equivalient to adding 0.5 -> floor ->
   -- -1 before multiply by stride. -1 is because C is 0 indexed where
   -- torch is 1 indexed.
   local output_map = pixel_coords[1]:clone():add(-0.5):floor():mul(stride[1])

   for d = 2,pixel_coords:size(1)-1 do 
      output_map:add(pixel_coords[d]:clone():add(-0.5):floor():mul(stride[d]))
   end
   -- Last dimension is indexed starting at 1 (so no -0.5)
   output_map:add(pixel_coords[-1]):floor()
   -- remove spurious out of bounds from output
   if mask then
      output_map[mask] = 1
   end

   offset:copy(output_map) -- conversion to long 

   return offset 
end

function offset_to_pixel_coords (offset, stride, pixels)
   local ndims = stride:size(1)
   local size = util.util.add_slices(ndims,offset:size())
   pixels = pixels or torch.Tensor(size)
   pixels:resize(size)

   local remainder = torch.Tensor(offset:size())
   -- put index back into torch 1 offset
   remainder:copy(offset):add(1)
   pixels[1]:copy(remainder):mul(1/stride[1]):ceil()
   
   for d = 2,ndims do 
      remainder:add(torch.add(pixels[d-1],-1):mul(-1*stride[d-1]))
      pixels[d]:copy(remainder):mul(1/stride[d]):floor() 
   end

   return pixels
end

