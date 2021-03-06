Class()
local pi = math.pi
local pi2 = math.pi*0.5

-- takes a table of images and a table of the corresponding masks
-- (binary visibility masks) and blends images into a single output
-- image using the masks. Warning: Images should be float.
function blend (images, masks)
   img_size = images[1]:size()
   usealpha = false
   if not masks then
      if img_size[1] == 4 then 
         usealpha = true
         allmask = images[1][4]:clone()
      else
         error("no alpha channel and no masks")
      end
   else
      allmask = masks[1]:clone()
   end
   for i = 2,#masks do
      if usealpha then 
         allmask:add(images[i][4])
      else
         allmask:add(masks[i])
      end
   end
   allmask:add(-(allmask:min()-1))
   allmask = allmask:typeAs(images[1]):add(-allmask:max()):abs()
   -- sum to 1
   allmask = allmask:clone():fill(1):cdiv(allmask)

   allmask[allmask:eq(math.huge)] = 0

   allmask_size = allmask:size()
   allmask:resize(util.util.add_slices(1,allmask_size))
   allmask = allmask:expand(util.util.add_slices(3,allmask_size))

   allimg = torch.cmul(images[1][{{1,3},{},{}}],allmask)
   for i = 2,#images do
      allimg:add(torch.cmul(images[i][{{1,3},{},{}}],allmask))
   end
   return allimg
end

function diagonal_to_height_width(diag,aspect_ratio)
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

-- inplace recenter angles to fall in range -pi,pi
function recenter_angles(angles)
   local sign = torch.sign(angles)
   angles:abs()

   local over   = angles:gt(pi)
   local n_over = over:sum()
   if (n_over > 0) then 
      printf(" - fixing %d pixels", n_over)
      -- find the number of times we have to subtract 2pi 
      local div = angles[over]:add(-pi):mul(1/(2*pi))
      div:apply(function (x) return math.modf(1+x) end)
      div:mul(-2*pi)
      -- subtract 2pi that number of times.
      angles[over] = angles[over]:add(div) 
   end
   -- put back the sign
   angles:cmul(sign)
end
