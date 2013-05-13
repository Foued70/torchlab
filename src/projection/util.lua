Class()
local pi = math.pi
local pi2 = math.pi*0.5

-- image is 3 x map dims
-- now map is 1D (this is faster)
function remap(img, index1D_and_mask)
   local out     = torch.Tensor()
   local index1D = index1D_and_mask.index1D
   local mask    = index1D_and_mask.mask
   local nelem   = index1D:nElement()
   local outsize = index1D:size()

   -- indexing has to be 1D, reshape only changes size locally
   -- TODO: allow multi dimensional index, in low level torch code
   if (index1D:nDimension() ~= 1) then
      index1D = index1D:reshape(index1D:nElement())
      mask    = mask:reshape(mask:nElement())
   end

   local ndim     = img:nDimension()

   if (ndim == 2) then
      -- single channel 2D

      out = img:reshape(img:nElement())[index1D]
      out[mask] = 0  -- erase out of bounds

      out:resize(outsize)

   elseif (ndim == 3) then
      -- n channel (RGB) (n x h x w)
      out:resize(img:size(1),nelem)

      for d = 1,img:size(1) do -- loop through channels
         local imgd = img[d]
         imgd:resize(imgd:nElement())

         out[d]       = imgd[index1D]
         out[d][mask] = 0

      end
      -- make output 3 x H x W
      out:resize(img:size(1),outsize[1],outsize[2])
   end
   return out
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
function make_mask(pixel_map,imgw,imgh)
   local xmap = pixel_map[1]
   local ymap = pixel_map[2]
   local mask = xmap:ge(1) + xmap:le(imgw)  -- out of bound in xmap
   mask = mask + ymap:ge(1) + ymap:le(imgh) -- out of bound in ymap
   -- reset mask to 0 and 1 (valid parts of mask must pass all 4
   -- tests).  We need the mask to reset bad pixels so the 1s are the
   -- out of bound pixels we want to replace
   mask[mask:ne(4)] = 1  -- out of bounds
   mask[mask:eq(4)] = 0  -- in bounds
   return mask
end

-- convert the x and y index into a single 1D offset (y * stride + x)
function make_index(pixel_map,mask,index1D,stride)
   local xmap = pixel_map[1]
   local ymap = pixel_map[2]
   index1D = index1D or torch.LongTensor(ymap:size())
   index1D:resizeAs(ymap)

   -- CAREFUL stride is in the original raw image not in the
   -- projection which can be scaled if xmap:size() is not equal to
   -- the size original image in which we index this function will
   -- return all the wrong values.
   stride = stride or xmap:size(2)

   -- CAREFUL must floor before multiplying by stride or does not make sense.
   -- -0.5 then floor is equivalient to adding 0.5 -> floor -> -1 before multiply by stride.
   local outmap = ymap:clone():add(-0.5):floor()

   -- ymap -1 so that multiply by stride makes sense (imgw is the stride)
   -- to map (1,1) ::  y = 0  * stride + x = 1 ==> 1
   -- to map (2,1) ::  y = 1  * stride + x = 1 ==> stride + 1 etc.

   outmap:mul(stride):add(xmap + 0.5)

   -- remove spurious out of bounds from output
   if mask then
      outmap[mask] = 1
   end

   index1D:copy(outmap) -- conversion to long rounds (+0.5 added above and floor)

   return index1D,stride
end

function index1D_to_xymap (index1D, stride, pixels)
   local size = {2} 
   for i = 1,index1D:dim() do table.insert(size, index1D:size(i)) end
   size = torch.LongTensor(size):storage()
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