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
-- forth between the offsets and coordinates (offset_to_coords,
-- coords_to_offset) and to quickly access the data based on an offset
-- Tensor (remap).  The offset tensor itself can have any number of
-- dimensions which shape the result of the remapping.

-- A coord (or coordinate) is an address in multiple parts (an x and y
-- coordinate or a height (h) and width (w) for 2D).  coords are
-- nCoords x nSamples because we often have to operate on coordinate
-- dimensions independently.  Storing coords nCoords by nSamples keeps
-- each dimension contiguous in memory speeding up operations such as
-- x = sin(x) + pi * cos(y) etc.  Other types of coordinates are 3D
-- x,y,z though we don't often store those densely in a tensor and a
-- CubeMap 6 x X x Y.

-- patch vs. image.  An image is a set of data of size nCoords or
-- nChannel x nCoords. A patch is a nCoords subset of an image.  For
-- example: grey_image is an array of numbers with dimensions
-- 100x100. patch_10x10 is a 10x10 set of these numbers starting at
-- image location (12,12) to (22,22).
-- 
-- +(1,1)------------------- ... ----+(1,100)
-- |                                 |
-- |    +(12,12)---+(12,22)          |
-- |    |          |                 |
-- |    |   patch  |                 |
-- |    |          |                 |
-- |    +(12,22)---+(22,22)          |
-- |                                 |
-- |            image                |
-- |                                 |
-- +(1,100)----------------- ... ----+(100,100)
-- 
-- The multi-channel color_image is 3x100x100 (RGB) or 4x100x100
-- (RGBA) and patch is 3x10x10.
-- 
-- A patch is non-contiguous.
-- 

-- using <offset> and (optional) <mask> remap <img> to
-- <out_image>.  <out_image> gets it's dimensions from <offset>
function remap(img, offset, mask, out_image)
   out_image          = out_image or torch.Tensor()
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
function mask_out_of_bounds(coords,max_dims,min_dims,mask)
   local nCoords = coords:size(1)
   if max_dims:size(1) ~= nCoords then
      error("not same number of dimensions as max_dims")
   end
   if not min_dims then
      min_dims = torch.ones(nCoords)
   end

   mask = mask or torch.ByteTensor(coords[1]:size())
   mask:resize(coords[1]:size()):zero()

   -- out of bounds check
   for d = 1,nCoords do 
      mask = mask + coords[d]:ge(min_dims[d]) + coords[d]:le(max_dims[d]) 
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
-- TODO could have multiple strides
function coords_to_offset(pixel_map,stride,mask,offset)
   local xmap = pixel_map[1]
   local ymap = pixel_map[2]
   offset = offset or torch.LongTensor(ymap:size())
   offset:resize(ymap:size())

   -- CAREFUL stride is in the original raw image not in the
   -- projection which can be scaled if xmap:size() is not equal to
   -- the size original image in which we index this function will
   -- return all the wrong values.
   stride = stride or error("must pass stride")

   -- CAREFUL must floor before multiplying by stride or does not make sense.
   -- -0.5 then floor is equivalient to adding 0.5 -> floor -> -1 before multiply by stride.
   local output_map = ymap:clone():add(-0.5):floor()

   -- ymap -1 so that multiply by stride makes sense (imgw is the stride)
   -- to map (1,1) ::  y = 0  * stride + x = 1 ==> 1
   -- to map (2,1) ::  y = 1  * stride + x = 1 ==> stride + 1 etc.

   output_map:mul(stride):add(xmap + 0.5)

   -- remove spurious out of bounds from output
   if mask then
      output_map[mask] = 1
   end

   offset:copy(output_map) -- conversion to long rounds (+0.5 added above and floor)

   return offset 
end

function offset_to_xymap (offset, stride, pixels)
   local size = util.util.add_slices(2,offset:size())
   pixels = pixels or torch.Tensor(size)
   pixels:resize(size)

   local xmap   = pixels[1]
   local ymap   = pixels[2]

   xmap:copy(offset):apply(function (x) return math.mod(x,stride) end)
   xmap[xmap:eq(0)] = stride
   ymap:copy(offset):mul(1/stride):ceil()

   return pixels
end

-- we need to operate in 2D offsets, index gives 1D offset
function offset_to_xy (index,row_width)
   local xy = torch.Tensor(2,index:size(1))
   local h = xy[1]
   local w = xy[2]

   h:copy(index)
   h:mul(1/row_width)
   h:ceil()

   w:copy(index)
   w:apply(function (x) return math.mod(x,row_width) end)
   w[w:eq(0)] = row_width

   return xy
end

function xy_to_offset(xy,row_width)

   local index = xy[1]:clone()
   index:add(-1)
   index:mul(row_width)
   index:add(xy[2])

   return index
end

function test_offset_to_xy (debug)
   -- 120 has long list of divisors, to test 1D to 2D
   local index = torch.range(1,120)
   local rw = torch.Tensor({2,3,4,5,6,8,10,12,15,20,24,30,40,60})
   local toterr = 0
   for r = 1,rw:size(1) do 
      local row_width = rw[r]
      local xy        = offset_to_xy(index,row_width)
      local index_out = xy_to_offset(xy,row_width)
      -- compute errors
      err = index - index_out
      err:abs()
      toterr = toterr + err:gt(0):sum()
      if debug then 
         print(xy[1]:resize(120/row_width,row_width))
         print(xy[2]:resize(120/row_width,row_width))
         print(index_out:resize(120/row_width,row_width))
      end 
   end
   printf("Errors: %d/%d",toterr,120*rw:size(1))
end


-- always step == 1
function pixel_coords_to_patch (pixel_coords_xy,pixel_coords_height,pixel_coords_width,patch_height,patch_width)
   local patch_xy = pixel_coords_xy:clone()
   
   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   local max_height = pixel_coords_height - patch_height + 1
   local max_width  = pixel_coords_width - patch_width + 1
   
   patch_xy[1]:add(1-ctr_height)
   patch_xy[2]:add(1-ctr_width)
   -- boundary
   patch_xy[patch_xy:lt(1)] = 1
   patch_xy[1][patch_xy[1]:gt(max_height)] = max_height
   patch_xy[2][patch_xy[2]:gt(max_width)] = max_width

   return patch_xy 
end

function test_pixel_coords_to_patch()
   local index = torch.range(1,120)
   local xy = index_to_xy(index,12)

   for _,wsh in pairs({3,5,7,9}) do 
      for _,wsw in pairs({3,5,7,9}) do 
         printf("window: %d,%d", wsh,wsw)
         local patch_xy = pixel_coords_to_patch(xy,10,12,wsh,wsw,1)

         print(patch_xy[1]:resize(10,12))
         print(patch_xy[2]:resize(10,12))
      end
   end
end

function pixel_coords_patch_to_image(patch_coords,patch_height,patch_width,image_height,image_width)

   local image_coords = patch_coords:clone()
   
   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   
   image_coords[1]:add(ctr_height-1)
   image_coords[2]:add(ctr_width-1)
   
   -- boundary already accounted for unless data is corrupt

   return image_coords

end

function patch_pixel_coords_to_image_pixel_coords(patch_pixel_coords,patch_h,patch_w)
   -- 1,1 goes to patch_h, patch_w
   local image_pixel_coords = patch_pixel_coords:clone()
   image_pixel_coords[1]:add(patch_h-1)
   image_pixel_coords[2]:add(patch_w-1)
   return image_pixel_coords
end

function test_patch_pixel_coords_to_image_pixel_coords()
   local index = torch.range(1,25)
   local xy = index_to_xy(index,5)

   local no_offset = patch_pixel_coords_to_image_pixel_coords(xy,1,1)
   print(no_offset[1]:resize(5,5))
   print(no_offset[2]:resize(5,5))
   local some_offset = patch_pixel_coords_to_image_pixel_coords(xy,10,100)
   print(some_offset[1]:resize(5,5))
   print(some_offset[2]:resize(5,5))
end

function get_centers(mask,patch_height,patch_width)

   local mask_height = mask:size(1)
   local mask_width  = mask:size(2)

   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   local n_height = mask_height - patch_height + 1
   local n_width  = mask_width  - patch_width + 1

   return mask:narrow(1,ctr_height,n_height):narrow(2,ctr_width,n_width)
end

function test_get_centers ()
   local mask = torch.range(1,120):resize(10,12)
   
   for _,r in pairs({3,5,7,9}) do 
      print(mask)
      print(get_centers(mask,r,r))
   end
end
