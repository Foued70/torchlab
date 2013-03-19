setfenv(1, setmetatable({}, {__index = _G}))

-- image is 3 x map dims
-- now map is 1D (this is faster)
function remap(img, map)
   local out    = torch.Tensor()
   local lookup = map.lookup_table
   local mask   = map.mask
   local nelem  = lookup:nElement()
   -- fast 1D remap
   if (lookup:nDimension() ~= 1) then
      print("ERROR map should be 1D Tensor")
      return nil
   end

   local ndim     = img:nDimension()

   if (ndim == 2) then
      -- single channel 2D

      local imgh = img:size(1)
      local imgw = img:size(2)

      img:resize(imgh * imgw)
      out = img[lookup] -- uses new indexing feature
      out[mask] = 0  -- erase out of bounds

      img:resize(imgh,imgw)
      out:resize(map.height,map.width)

   elseif (ndim == 3) then
      -- n channel (RGB) (n x h x w)
      printf("output image size: %d x %d", img:size(1), nelem)
      out:resize(img:size(1),nelem)

      for d = 1,img:size(1) do -- loop through channels
         local imgd = img[d]
         imgd:resize(imgd:size(1)*imgd:size(2))

         out[d] = imgd[lookup]
         out[d][mask] = 0

      end
      -- make output 3 x H x W
      out:resize(img:size(1),map.height,map.width)
   end
   return out
end

-- compute angles using spherical geometry 
local noneuclidean = false

function make_diagonal(hfov,vfov,mapw)
   -- +++++
   -- find dimension of the map
   -- +++++
   local maph = mapw * (vfov/hfov)

   -- create horizontal (lambda) and vertical angles (phi)
   -- (equirectangular) x,y lookup in spherical map from
   -- -radians,radians at resolution mapw and maph
   local lambda = torch.linspace(-hfov,hfov,mapw)
   local phi    = torch.linspace(-vfov,vfov,maph)

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
      collectgarbage() -- not such a big deal as we are using expand
   else
      -- normal pythagorean theorem
      local xsqr = lambda:clone():cmul(lambda):resize(1,mapw):expand(maph,mapw) 
      local ysqr = phi:clone():cmul(phi):resize(maph,1):expand(maph,mapw)
      theta = torch.add(xsqr,ysqr)
      theta:sqrt()
      xsqr = nil
      ysqr = nil
      collectgarbage() -- not such a big deal as we are using expand
   end
   return theta, lambda, phi
end

-- make mask for out of bounds values
function make_mask(xmap,ymap,imgw,imgh)
   local mask = xmap:ge(1) + xmap:lt(imgw)  -- out of bound in xmap
   mask = mask + ymap:ge(1) + ymap:lt(imgh) -- out of bound in ymap
   -- reset mask to 0 and 1 (valid parts of mask must pass all 4
   -- tests).  We need the mask to reset bad pixels so the 1s are the
   -- out of bound pixels we want to replace
   mask[mask:ne(4)] = 1  -- out of bounds
   mask[mask:eq(4)] = 0  -- in bounds
   return mask
end

-- convert the x and y index into a single 1D offset (y * stride + x)
function make_index(xmap,ymap,mask)

   local stride = xmap:size(2)

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

   local index_map = outmap:long() -- round (+0.5 above) and floor

   -- make 1D
   index_map:resize(index_map:nElement())

   return index_map
end

return (getfenv())
