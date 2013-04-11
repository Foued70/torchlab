Class()

local piover2 = math.pi*0.5

-- thoby constants
local k1 = 1.47
local k2 = 0.713

-- rectilinear max fov in radians
local max_rad_rectilinear = 1

-- image is 3 x map dims
-- now map is 1D (this is faster)
function remap(img, map)
   local out    = torch.Tensor()
   local lookup = map.lookup_table
   local mask   = map.mask
   local nelem  = lookup:nElement()
   -- fast 1D remap
   if (lookup:nDimension() ~= 1) then
      log.error("map should be 1D Tensor")
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
      log.tracef("output image size: %d x %d", img:size(1), nelem)
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



function compute_diagonal_fov(diagonal_normalized,lens_type,params)
   local convert = false
   if (type(diagonal_normalized) == "number") then 
      diagonal_normalized = torch.Tensor({diagonal_normalized})
      convert = true
   end
   local dfov
   if (lens_type == "rectilinear") then
      dfov = torch.atan(diagonal_normalized)   -- diag in rad
   elseif (lens_type == "scaramuzza_r2t") then
      log.trace(" -- using scaramuzza calibration")
      local d2 = diagonal_normalized:clone()
      dfov = torch.Tensor(d2:size()):fill(params[-1])
      for i = params:size(1)-1,1,-1 do 
         dfov = dfov + d2 * params[i]
         d2:cmul(diagonal_normalized)
      end
      -- using ideal thoby to compute fov for scaramuzza's original
      -- code which does not solve for theta. Can remove this eventually.
   elseif (lens_type == "thoby") or (lens_type == "scaramuzza") then
      log.trace(" -- using lens type: thoby")
      -- thoby : theta = asin((r/f)/(k1 * f))/k2
      if (diagonal_normalized:max() > k1) then 
         log.error("diagonal too large for thoby")
      else
         dfov = torch.asin(diagonal_normalized/k1)/k2
      end
   elseif (lens_type == "equal_angle") then
      dfov = diagonal_normalized

   elseif (lens_type =="equal_area") then
      if( diagonal_normalized:max() <= 2 ) then
         dfov = 2 * torch.asin( 0.5 * diagonal_normalized )
      end
      if( torch.sum(dfov:eq(0)) > 0) then
         error( "equal-area FOV too large" )
      end

   elseif (lens_type == "stereographic") then
      dfov = 2 * torch.atan(diagonal_normalized*0.5 )

   elseif (lens_type == "orthographic") then
      if( diagonal_normalized:max() <= 1 ) then
         dfov = torch.asin( diagonal_normalized )
      end
      if( torch.sum(dfov:eq(0)) > 0) then
         error( "orthographic FOV too large" )
      end

   else
      error("don't understand self lens model requested")
   end
   if convert then 
      dfov = dfov[1]
   end
   return dfov
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

-- based on final projection type create a map of angles which need to
-- be looked up in the original image.
function projection_to_sphere (fov,hfov,vfov,mapw,maph,aspect_ratio,proj_type)
   
   local lambda,phi,theta_map, output_map
   
   if (proj_type == "rectilinear") then
      -- limit the fov to roughly 120 degrees
      if fov > max_rad_rectilinear then
         fov = max_rad_rectilinear
         vfov,hfov = derive_hw(fov,aspect_ratio)
      end
      -- set up size of the output table
      local drange = torch.tan(fov)
      local vrange,hrange = derive_hw(drange,aspect_ratio)
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = make_pythagorean_map(lambda,phi)
      theta_map = output_map:clone():atan()
   elseif (proj_type == "cylindrical") then
      -- limit the vfov to roughly 120 degrees
      if vfov > 1 then
         vfov = 1
         maph = mapw * (vfov / hfov)
      end
      -- set up size of the output table
      local vrange = torch.tan(vfov)
      local hrange = hfov
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = make_pythagorean_map(lambda,phi)
      lambda:atan()
      theta_map = make_pythagorean_map(lambda,phi)
   elseif (proj_type == "cylindrical_vert") then
      -- limit the hfov to roughly 120 degrees
      if hfov > 1 then
         hfov = 1
         mapw = maph * (hfov/vfov)
      end
      -- set up size of the output table
      local vrange = vfov
      local hrange = torch.tan(hfov)
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = make_pythagorean_map(lambda,phi)
      phi:atan()
      theta_map = make_pythagorean_map(lambda,phi)
   else
      -- default is to project to sphere
      -- create horizontal (lambda) and vertical angles (phi) x,y lookup
      --  in spherical map from -radians,radians at resolution mapw and
      --  maph.  Equal steps in angles.
      lambda = torch.linspace(-hfov,hfov,mapw)
      phi    = torch.linspace(-vfov,vfov,maph)
      output_map = make_pythagorean_map(lambda,phi)
      theta_map  = output_map
   end
   return lambda, phi, theta_map, output_map, mapw, maph

end

function sphere_to_camera(theta_map,lens_type,params)
   local r_map = theta_map:clone() -- copy

   if (lens_type == "rectilinear") then
      -- rectilinear : (1/f) * r' = tan(theta)
      r_map:tan()
   elseif (lens_type == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      r_map:mul(k2):sin():mul(k1)
   elseif (lens_type == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      r_map:mul(0.5):tan():mul(2)
   elseif (lens_type == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      r_map:sin()
   elseif (lens_type == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      r_map:mul(0.5):sin():mul(2)
   elseif (lens_type == "scaramuzza") then
      local theta     = theta_map:clone()
      -- we use a positive angle from optical center, but scaramuzza
      -- uses a negative offset from focal point.
      theta:add(-piover2) 
      local theta_pow = theta:clone()
      r_map:fill(params[-1]) -- rho = invpol[0]
      
      for i = params:size(1)-1,1,-1 do          
         r_map:add(theta_pow * params[i]) -- coefficients of inverse poly.
         theta_pow:cmul(theta)        -- powers of theta
      end 
   elseif lens_type:gmatch("scaramuzza_r2t") then
      local theta_pow = theta_map:clone()
      
      r_map:fill(params[-1]) -- rho = invpol[0]
      
      for i = params:size(1)-1,1,-1 do          
         r_map:add(theta_pow * params[i]) -- coefficients of inverse poly.
         theta_pow:cmul(theta_map)        -- powers of theta
      end 
   else
      log.error("don't understand lens_type")
      return nil
   end
   return r_map
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
function make_index(xmap,ymap,mask,stride)

   -- CAREFUL stride is in the original raw image not in the
   -- projection which can be scaled if xmap:size() is not equal to
   -- the size original image in which we index this function will
   -- return all the wrong values.
   if not stride then 
      stride = xmap:size(2)
   end

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