Class()
local pi = math.pi
local pi2 = math.pi*0.5

-- image is 3 x map dims
-- now map is 1D (this is faster)
function remap(img, index1D, stride, mask, out_image)
   out_image          = out_image or torch.Tensor()
   local index_n_elem = index1D:nElement()
   local index_size   = index1D:size()
   local index_dim    = index1D:nDimension()
   local image_dim    = img:nDimension()

   -- indexing has to be 1D, reshape only changes the size locally
   -- TODO: allow multi dimensional index, in low level torch code
   if (index_dim ~= 1) then
      index1D = index1D:reshape(index_n_elem)
      if mask then 
         if (mask:nElement() ~= index_n_elem) then
            error("mask not same size as index1D")
         else
            mask    = mask:reshape(index_n_elem)
         end
      end
   end
   
   if (image_dim == 2) then
      -- single channel 2D, out_image has same dimensions as index
      out_image:resize(index_n_elem)
      -- flatten copy of img, apply the index and copy into out_image 
      out_image = img:reshape(img:nElement())[index1D]
      if mask then 
         out_image[mask] = 0  -- erase out of bounds values
      end
      -- unflatten out_image to the original dimensions of the index
      out_image:resize(index_size)

   elseif (image_dim == 3) then
      local n_slices = img:size(1)
      local input_elements_per_slice = img[1]:nElement()

      -- n channel (RGB) (n x h * w)
      out_image:resize(n_slices,index_n_elem)

      for d = 1,n_slices do -- loop through channels one at a time
         local imgd = img[d]
         -- flatten
         imgd:resize(input_elements_per_slice)

         out_image[d]    = imgd[index1D]
         if mask then 
            out_image[d][mask] = 0 -- erase out of bounds values
         end
      end
      -- make output 3 x H x W (or n_slices by whatever)
      out_size = util.util.add_slices(n_slices, index_size)
      out_image:resize(out_size)
   end
   return out_image
end

function derive_hw(diag,aspect_ratio)
   -- horizontal and diagonal fov's are useful for size of our lookup
   -- table.  Using normal (euclidean) pythagorean theorem to compute
   -- w and h from the aspect ratio and the diagonal which doesn't
   -- feel right but gives the expected result as opposed to the
   -- non-euclidean cos(c) = cos(a)cos(b)
   local h = diag/math.sqrt(aspect_ratio*aspect_ratio + 1)
   local w = h*aspect_ratio
   return h,w
end


-- given an xrange and a yrange (lambda, phi) compute the 2D map of
-- diagonal distances these x and y values cover.
function make_pythagorean_map (lambda,phi,noneuclidean)
   local mapw = lambda:size(1)
   local maph = phi:size(1)
   local theta
   if noneuclidean then
      -- non-euclidean pythagorean
      local cosx = lambda:clone():cos():resize(1,mapw):expand(maph,mapw)
      local cosy = phi:clone():cos():resize(maph,1):expand(maph,mapw)
      -- pythagorean theorem on a unit sphere is cos(c) = cos(a)cos(b)
      -- c = arccos(cos(a)cos(b))
      theta = torch.cmul(cosx,cosy):acos()
      cosx = nil
      cosy = nil
   else
      -- normal pythagorean theorem
      local xsqr = lambda:clone():cmul(lambda):resize(1,mapw):expand(maph,mapw) 
      local ysqr = phi:clone():cmul(phi):resize(maph,1):expand(maph,mapw)
      theta = torch.add(xsqr,ysqr)
      theta:sqrt()
      xsqr = nil
      ysqr = nil
   end
   return theta
end

-- make mask for out of bounds values
function make_mask(pixel_map,imgw,imgh,mask)
   local xmap = pixel_map[1]
   local ymap = pixel_map[2]
   mask = mask or torch.ByteTensor(xmap:size())
   mask:resize(xmap:size())
   mask:copy(xmap:ge(1) + xmap:le(imgw))    -- out of bound in xmap
   mask = mask + ymap:ge(1) + ymap:le(imgh) -- out of bound in ymap
   -- reset mask to 0 and 1 (valid parts of mask must pass all 4
   -- tests).  We need the mask to reset bad pixels so the 1s are the
   -- out of bound pixels we want to replace
   mask[mask:ne(4)] = 1  -- out of bounds
   mask[mask:eq(4)] = 0  -- in bounds
   return mask
end

-- convert the x and y index into a single 1D offset (y * stride + x)
function make_index(pixel_map,stride,mask,index1D)
   local xmap = pixel_map[1]
   local ymap = pixel_map[2]
   index1D = index1D or torch.LongTensor(ymap:size())
   index1D:resize(ymap:size())

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

   index1D:copy(output_map) -- conversion to long rounds (+0.5 added above and floor)

   return index1D 
end

function index1D_to_xymap (index1D, stride, pixels)
   local size = util.util.add_slices(2,index1D:size())
   pixels = pixels or torch.Tensor(size)
   pixels:resize(size)

   local xmap   = pixels[1]
   local ymap   = pixels[2]

   xmap:copy(index1D):apply(function (x) return math.mod(x,stride) end)
   xmap[xmap:eq(0)] = stride
   ymap:copy(index1D):mul(1/stride):ceil()

   return pixels
end

-- inplace recenter angles to fall in range -pi,pi
function recenter_angles(angles)
   local sign = torch.sign(angles)
   angles:abs()

   local over   = angles:gt(pi)
   local n_over = over:sum()
   if (n_over > 0) then
      printf(" - fixing %d pixels", n_over)
      angles[over] = angles[over]:add(-2*pi)
   end
   angles:cmul(sign)
end