require 'image'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Texture fill areas of image')
cmd:text()
cmd:text('Options')
cmd:option('-imagefile', '', 'image with areas to fill')
cmd:option('-maskfile', '', '(optional) mask of areas to be filled')
cmd:option('-outfile', 'filled', 'basename for filled image')
cmd:option('-bs',10,'batchsize - number of image patches to be processed at once')
cmd:option('-ws',25,'windowsize - size in pixels of a patch')
cmd:option('-lh',256,'lookup_size_h - height of window in which to look up')
cmd:option('-lw',256,'lookup_size_w - width of window in which to look up')

cmd:text()

-- parse input params
local params = cmd:parse(arg)

local image_file  = params.imagefile
local mask_file   = params.maskfile
local out_file    = params.outfile
local batch_size = params.bs
local window_size = params.ws
local ws = window_size
-- size of area in which to find matching patches (in pixels)
local lookup_size_h = params.lh
local lookup_size_w = params.lw

if not paths.filep(image_file) then 
   error(string.format("-imagefile %s not found",image_file))
end

local img_rgb = image.load(image_file)

local mask
if mask_file and paths.filep(mask_file) then 
   printf(" - attempt to load mask file %s", mask_file)
   mask = image.load(mask_file)
elseif img_rgb:size(1) == 4 then 
   print(" - using alpha channel as mask")
   mask = img_rgb[4]
   img_rgb = img_rgb:narrow(1,1,3)
else
   print(" - making mask of all completely black pixels in the image")
   mask = img_rgb[1]:eq(0)
   mask = mask + img_rgb[2]:eq(0)
   mask = mask + img_rgb[3]:eq(0)
   mask = mask:eq(3):double()
end

-- three levels of context
-- pixel  : no context only the pixel
-- patch  : a window around the pixel (eg. 5x5)
-- lookup : a set of windows (eg. 10x10 of 5x5)

local timer = ui.Timer.new()

local n_random_patches = 2

local ctr = math.ceil(ws*0.5)

local channel_mix  = torch.Tensor({40,20,20})

mask[mask:gt(0)] = 1 -- binarize
local tofill       = mask:clone()

local image_h      = img_rgb:size(2)
local image_w      = img_rgb:size(3)
local pano_lab     = image.rgb2lab(img_rgb)

-- views into the mask which gets up dated
local mask_patches = tofill:unfold(1,ws,1):unfold(2,ws,1)

local mps          = mask_patches:size()

-- make data 
local pano_patches = pano_lab:unfold(2,ws,1):unfold(3,ws,1)
local pano_lookup  = pano_lab:unfold(2,lookup_size_h,1):unfold(3,lookup_size_w,1)
local mask_lookup  = mask:unfold(1,lookup_size_h,1):unfold(2,lookup_size_w,1)
local pls          = pano_lookup:size()

-- flatten
local total_patches = mps[1]*mps[2]

-- we need to operate in 2D offsets, index gives 1D offset
function index_to_hw (index,row_width)
   local hw = torch.Tensor(2,index:size(1))
   local h = hw[1]
   local w = hw[2]

   h:copy(index)
   h:mul(1/row_width)
   h:ceil()

   w:copy(index)
   w:apply(function (x) return math.mod(x,row_width) end)
   w[w:eq(0)] = row_width

   return hw
end

function hw_to_index(hw,row_width)

   local index = hw[1]:clone()
   index:add(-1)
   index:mul(row_width)
   index:add(hw[2])

   return index
end

function test_index_to_hw (debug)
   -- 120 has long list of divisors, to test 1D to 2D
   local index = torch.range(1,120)
   local rw = torch.Tensor({2,3,4,5,6,8,10,12,15,20,24,30,40,60})
   local toterr = 0
   for r = 1,rw:size(1) do 
      local row_width = rw[r]
      local hw        = index_to_hw(index,row_width)
      local index_out = hw_to_index(hw,row_width)
      -- compute errors
      err = index - index_out
      err:abs()
      toterr = toterr + err:gt(0):sum()
      if debug then 
         print(hw[1]:resize(120/row_width,row_width))
         print(hw[2]:resize(120/row_width,row_width))
         print(index_out:resize(120/row_width,row_width))
      end 
   end
   printf("Errors: %d/%d",toterr,120*rw:size(1))
end

-- always step == 1
function px_to_patch (px_hw,px_height,px_width,patch_height,patch_width)
   local patch_hw = px_hw:clone()
   
   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   local max_height = px_height - patch_height + 1
   local max_width  = px_width - patch_width + 1
   
   patch_hw[1]:add(1-ctr_height)
   patch_hw[2]:add(1-ctr_width)
   -- boundary
   patch_hw[patch_hw:lt(1)] = 1
   patch_hw[1][patch_hw[1]:gt(max_height)] = max_height
   patch_hw[2][patch_hw[2]:gt(max_width)] = max_width

   return patch_hw 
end

function test_px_to_patch()
   local index = torch.range(1,120)
   local hw = index_to_hw(index,12)

   for _,wsh in pairs({3,5,7,9}) do 
      for _,wsw in pairs({3,5,7,9}) do 
         printf("window: %d,%d", wsh,wsw)
         local patch_hw = px_to_patch(hw,10,12,wsh,wsw,1)

         print(patch_hw[1]:resize(10,12))
         print(patch_hw[2]:resize(10,12))
      end
   end
end

function patch_to_px(patch_hw,patch_height,patch_width,px_height,px_width)

   local px_hw = patch_hw:clone()
   
   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   
   px_hw[1]:add(ctr_height-1)
   px_hw[2]:add(ctr_width-1)
   
   -- boundary already accounted for unless data is corrupt

   return px_hw

end

function patch_px_to_image_px(patch_px,patch_h,patch_w)
   -- 1,1 goes to patch_h, patch_w
   local image_px = patch_px:clone()
   image_px[1]:add(patch_h-1)
   image_px[2]:add(patch_w-1)
   return image_px
end

function test_patch_px_to_image_px()
   local index = torch.range(1,25)
   local hw = index_to_hw(index,5)

   local no_offset = patch_px_to_image_px(hw,1,1)
   print(no_offset[1]:resize(5,5))
   print(no_offset[2]:resize(5,5))
   local some_offset = patch_px_to_image_px(hw,10,100)
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

-- make list of patches with sum > 0 and center pixel == 0

-- operation is sum of all windows (which we can easily make window
-- apply function for)

-- uses convolution with a particular kernel to compute the edge pixels quickly in a single sweep.
function find_patches (mask,ws)
   sys.tic()
   -- dilation kernel
   local kernel = torch.ones(ws,ws)
   -- center of match must be zero so set center kernel to -(number_of_offcenter_pixels)
   kernel[ctr][ctr] = -((ws * ws) - 1)
   
   -- apply dilation which is fast even on large input
   mask_dilated = torch.conv2(mask,kernel,'F')
   mask_dilated = mask_dilated:narrow(1,ctr,image_h):narrow(2,ctr,image_w)
   mask_dilated[mask_dilated:lt(0)] = 0 -- remove negative values from mask
   
   local byte_index = mask_dilated:gt(0)
   local found = byte_index:sum()
   
   if (found == 0) then 
      print("Filled all patches")
      return nil
   end
      
   local mask_range = torch.range(1,image_h*image_w):resize(image_h,image_w)
   local mask_index = mask_range[byte_index] 
   
   -- sort by most neighbors
   local y,i = mask_dilated[byte_index]:sort(true)
   
   if i:size(1) > batch_size then 
      i = i:narrow(1,1,batch_size)
   end

   mask_index = mask_index[i]

   local n_patches = mask_index:size(1)

   printf(" - found %d patches of %d from %d to %d neighbors in %2.4fs",
          n_patches, mask:sum(), y[1], y[n_patches], sys.toc())

   return mask_index

end

function display_mask_index (mask_index,h,w,win,zoom)
   local display_mask = torch.zeros(h*w)
   display_mask[mask_index:long()] = 1
   display_mask:resize(h,w)
   win = image.display{image=display_mask,win=win,zoom=zoom}
   return display_mask, win
end

function fill_patches (mask_index,debug)

   collectgarbage()

   local timer = torch.Timer.new()
   
   local t0 = timer:time()     
   local n_patches = mask_index:size(1)
   local mask_hw   = index_to_hw(mask_index,image_w)
   local patch_hw  = px_to_patch(mask_hw,image_h,image_w,ws,ws)
   local lookup_hw = px_to_patch(mask_hw,image_h,image_w,
                                 lookup_size_h,lookup_size_w)

   local t1 = timer:time()
   local s1 = t1.real - t0.real
   local s2 = 0 
   local s3 = 0
   local s4 = 0
   local s5 = 0
   local s6 = 0
   sys.tic()
   for pi = 1,mask_index:size(1) do
      
      local t2 = timer:time()
      local mask_h  = mask_hw[1][pi]
      local mask_w  = mask_hw[2][pi] 

      local patch_h = patch_hw[1][pi]
      local patch_w = patch_hw[2][pi]

      -- find window in which to lookup patches
      local lookup_h = lookup_hw[1][pi]
      local lookup_w = lookup_hw[2][pi]

      if debug then
         printf("Processing patch: %d %d", patch_h,patch_w)
      end

      -- extract patch
      local patch = pano_patches[{{},patch_h,patch_w,{},{}}]:clone()
      
      -- prepare mask patch we are copying into (gets updated as we go)
      local patch_mask = mask_patches[{patch_h,patch_w,{},{}}]:clone()
      patch_mask:cmul(image.gaussian(ws))      
      
      if debug then
         printf("Loading %d,%d lookup table of patches",lookup_h,lookup_w)
      end

      local lookup = pano_lookup[{{},lookup_h,lookup_w,{},{}}]:unfold(2,ws,1):unfold(3,ws,1)

      -- mask of lookup we are coping from (only original image)
      local lookup_mask = mask_lookup[{lookup_h,lookup_w,{},{}}]:unfold(1,ws,1):unfold(2,ws,1)

      local t3 = timer:time()
      s2 = s2 +  t3.real - t2.real

      -- TODO: make get_centers handle the 3 channels
      local lookup_centers1 = get_centers(pano_lookup[{1,lookup_h,lookup_w,{},{}}],ws,ws)
      local lookup_centers2 = get_centers(pano_lookup[{2,lookup_h,lookup_w,{},{}}],ws,ws)
      local lookup_centers3 = get_centers(pano_lookup[{3,lookup_h,lookup_w,{},{}}],ws,ws)

      local lookup_mask_centers = get_centers(mask_lookup[{lookup_h,lookup_w,{},{}}],ws,ws):clone()
      lookup_mask_centers[lookup_mask_centers:lt(1)] = ws * ws

      local n_lookup_h = lookup:size(2)
      local n_lookup_w = lookup:size(3)
      local n_lookup_tot = n_lookup_h * n_lookup_w

      -- big fake replicated patches
     
      -- at least do one row at a time
      local patch1 = patch:narrow(1,1,1):expand(n_lookup_w,ws,ws)
      local patch2 = patch:narrow(1,2,1):expand(n_lookup_w,ws,ws)
      local patch3 = patch:narrow(1,3,1):expand(n_lookup_w,ws,ws)
      
      patch_mask = patch_mask:reshape(1,ws,ws):expand(n_lookup_w,ws,ws) 
      local distances  = torch.Tensor(n_lookup_h,n_lookup_w)

      local t4 = timer:time()
      s3 = s3 + t4.real - t3.real

      for c = 1,n_lookup_h do
         -- find overlap of masks
         local row_mask = torch.cmul(patch_mask,lookup_mask[c])

         -- compute L1 distance (so much faster that squaring)
         local dst1 = lookup[1][c] - patch1
         local dst2 = lookup[2][c] - patch2
         local dst3 = lookup[3][c] - patch3

         dst1:abs():mul(channel_mix[1])
         dst2:abs():mul(channel_mix[2])
         dst3:abs():mul(channel_mix[3])

         -- pixel distance w/out regard to mask (3 color channels)
         local dst = dst1 + dst2 + dst3
            
         -- apply mask and gaussian (center weight)
         dst:cmul(row_mask) -- multiply with overlap mask
      
         -- pixel distances
         dst = dst:reshape(dst:size(1),ws*ws) -- sum over patch
         distances[c]:copy(dst:sum(2):squeeze())

         -- more overlap
         row_mask:resize(row_mask:size(1),ws*ws)
         local overlap = row_mask:sum(2):squeeze()
         overlap:add(-(ws*ws)):mul(-1)
         distances[c]:add(overlap)

         
      end

      -- center pixel of target mask must equal 1
      -- make sure center of matched patch is not zero
      distances:cmul(lookup_mask_centers) 

      distances:resize(n_lookup_tot)

      local n_found = distances:lt(ws * ws):sum()
      n_found = n_found > n_random_patches and n_random_patches or n_found
      n_found = n_found > 1 and n_found or 1

      local t5 = timer:time()
      s4 = s4 + t5.real - t4.real
      -- randomize the input to get around bad implementation of torch.sort()
      local rand_index = torch.randperm(distances:size(1)):long()
      local rand_dist  = distances[rand_index]
      local dst,dsti   = rand_dist:sort()

      local t6 = timer:time()
      s5 = s5 + t6.real - t5.real
      -- matching patch index in lookup table
      local patch_index = rand_index[dsti:narrow(1,1,n_found)]

      local lookup_patch_hw = index_to_hw(patch_index,n_lookup_w)

      -- pick random patch from top n
      local ri = math.random(n_found)

      if debug_visual then
         -- display patch + n_random_patches best matches
         local dpatches = torch.Tensor(n_found+1,3,ws,ws):zero()
         dpatches[1]:copy(patch)
         local j = 2
         for i = 1,n_found do 
            dpatches[j]:copy(lookup[{{},lookup_patch_hw[1][i],lookup_patch_hw[2][i],{},{}}])
            j = j + 1
         end
         image.display{image=dpatches,zoom=5,nrow=31}
      end

      lookup_patch_hw = lookup_patch_hw:narrow(2,ri,1)

      local pixel_in_lookup = patch_to_px(lookup_patch_hw,ws,ws,n_lookup_h,n_lookup_w)

      lookup_patch_hw = lookup_patch_hw:squeeze()

      local pixel_in_image = patch_px_to_image_px(pixel_in_lookup,lookup_h,lookup_w)
      pixel_in_image = pixel_in_image:squeeze()

      -- copy pixel data
      img_rgb[{{},mask_h,mask_w}] = img_rgb[{{},pixel_in_image[1],pixel_in_image[2]}]

      if (img_rgb[{{},mask_h,mask_w}]:sum() == 0) then 
         px = img_rgb[{{},mask_h,mask_w}]
         printf(" copied: %f %f %f",px[1],px[2],px[3])
         printf(" center: %f %f %f",
             lookup_centers1[lookup_patch_hw[1]][lookup_patch_hw[2]],
             lookup_centers2[lookup_patch_hw[1]][lookup_patch_hw[2]],
             lookup_centers3[lookup_patch_hw[1]][lookup_patch_hw[2]])
         print("** COPIED zero value **")
         print(mask_patches[lookup_patch_hw[1]][lookup_patch_hw[2]])
         printf("index: %d",ri)
         print(distances[ri])
      end
      -- update mask
      tofill[mask_h][mask_w] = 1
      local t7 = timer:time()
      s6 = s6 + t7.real - t6.real
   end
   printf(" - processed %d patches in %2.4fs", n_patches, sys.toc())
   print("time:")
   printf(" - setup                %2.4fs",s1)
   print("in loop:")
   printf(" - extract data         %2.4fs",s2)
   printf(" - get_centers, expand  %2.4fs",s3)
   printf(" - compute distances    %2.4fs",s4)
   printf(" - sort                 %2.4fs",s5)
   printf(" - apply                %2.4fs",s6)
end

mask_index = find_patches(tofill,ws)

while mask_index do 
   fill_patches(mask_index)
   mask_index = find_patches(tofill,ws)
   collectgarbage()
   -- image.save("tmp.png",img_rgb)
   -- win = image.display{win=win,image={img_rgb},zoom=3} 
end
