-- Class()

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
cmd:option('-ws',15,'windowsize - size in pixels of a patch')
cmd:option('-lh',128,'lookup_h - height of window in which to look up')
cmd:option('-lw',128,'lookup_w - width of window in which to look up')

cmd:text()
print("init"); log.tic()
-- parse input params
params = cmd:parse(process.argv)

image_file  = params.imagefile
mask_file   = params.maskfile
out_file    = params.outfile
out_file    = out_file .. ".png"

batch_size = params.bs
window_size = params.ws
ws = window_size
-- size of area in which to find matching patches (in pixels)
lookup_h = params.lh
lookup_w = params.lw

if not util.fs.is_file(image_file) then 
   error(string.format("-imagefile %s not found",image_file))
end

w = image.Wand.new(image_file)

img_rgb = w:toTensor("double", "RGBA", "DHW")

global_lookup_mask = nil
if mask_file and util.fs.is_file(mask_file) then 
   printf(" - attempt to load mask file %s", mask_file)
   global_lookup_mask = image.load(mask_file)
elseif img_rgb:size(1) == 4 and img_rgb[4]:sum() > 0 then 
   print(" - using alpha channel as mask")
   global_lookup_mask = img_rgb[4]
   img_rgb = img_rgb:narrow(1,1,3)
else
   print(" - making mask of all completely black pixels in the image")
   global_lookup_mask = img_rgb[1]:eq(0)
   global_lookup_mask = global_lookup_mask + img_rgb[2]:eq(0)
   global_lookup_mask = global_lookup_mask + img_rgb[3]:eq(0)
   global_lookup_mask = global_lookup_mask:lt(3):double()
end


-- three levels of context
-- pixel  : no context only the pixel
-- patch  : a window around the pixel (eg. 5x5)
-- lookup : a set of windows (eg. 10x10 of 5x5)

n_random_patches = 2

ctr = math.ceil(ws*0.5)

channel_mix  = torch.Tensor({40,20,20})

-- and alpha channel of 1 mean full visibility. We fill everythign with an alpha less than 1
global_lookup_mask[global_lookup_mask:lt(1)] = 0 -- binarize but mask is still double
global_tofill_mask = global_lookup_mask:clone()


printf("filling %d of %d pixels", global_tofill_mask:eq(0):sum(), global_tofill_mask:nElement())

-- from whence the offsets are copied (C indexing)
global_mask_range   = torch.range(1,global_tofill_mask:nElement()):resize(global_tofill_mask:size())
global_mask_stride  = global_tofill_mask:stride()

image_h      = img_rgb:size(2)
image_w      = img_rgb:size(3)
pano_lab     = w:toTensor("double", "LAB", "DHW")

-- views into the mask which gets up dated
tofill_patches = global_tofill_mask:unfold(1,ws,1):unfold(2,ws,1)
mps            = tofill_patches:size()

-- make data 
pano_patches   = pano_lab:unfold(2,ws,1):unfold(3,ws,1)
pano_lookup    = pano_lab:unfold(2,lookup_h,1):unfold(3,lookup_w,1)
global_lookup_mask_windows = global_lookup_mask:unfold(1,lookup_h,1):unfold(2,lookup_w,1)
pls            = pano_lookup:size()

-- flatten
total_patches = mps[1]*mps[2]

printf('init %dms', log.toc())


-- make list of patches with sum > 0 and center pixel == 0

-- operation is sum of all windows (which we can easily make window
-- apply function for)

-- uses convolution with a particular kernel to compute the edge pixels quickly in a single sweep.
function find_patches (tofill_mask,ws)
   print("find_patches")
   -- dilation kernel
   local kernel = torch.ones(ws,ws)
   -- center of match must be zero so set center kernel to -(number_of_offcenter_pixels)
   kernel[ctr][ctr] = -((ws * ws) - 1)
   
   -- apply dilation which is fast even on large input
   local mask_dilated = torch.conv2(tofill_mask,kernel,'F') -- full convolution
   -- narrow back to same size as images
   mask_dilated = mask_dilated:narrow(1,ctr,image_h):narrow(2,ctr,image_w)
   -- mask_dilated[mask_dilated:lt(0)] = 0 -- remove negative values from mask
   
   local byte_index = mask_dilated:gt(0)
   local found = byte_index:sum()
   
   if (found == 0) then 
      print("Filled all patches")
      return nil
   end

   local mask_offsets = global_mask_range[byte_index] 
   
   -- sort by most neighbors
   local y,i = mask_dilated[byte_index]:sort(true)
   
   if i:size(1) > batch_size then 
      i = i:narrow(1,1,batch_size)
   end

   -- select by index
   mask_offsets = mask_offsets[i]

   local n_patches = mask_offsets:size(1)

   printf(" - found %d patches of %d from %d to %d neighbors",
          n_patches, tofill_mask:nElement() - tofill_mask:sum(), y[1], y[n_patches])

   -- cleanup 
   byte_index = nil
   mask_dilated = nil
   collectgarbage()
   printf("find_patches %dms", log.toc())

   return mask_offsets

end

function display_mask_offsets (mask_offsets,h,w,win,zoom)
   local display_mask = torch.zeros(h*w)
   display_mask[mask_offsets:long()] = 1
   display_mask:resize(h,w)
   win = image.display{image=display_mask,win=win,zoom=zoom}
   return display_mask, win
end

function fill_patches (mask_offsets,lookup_mask_windows,debug)

   collectgarbage()

   local timer = torch.Timer.new()
   
   local t0 = timer:time()     
   local n_patches = mask_offsets:size(1)
   mask_xy   = util.addr.offset_to_pixel_coords(mask_offsets,global_mask_stride)
   patch_xy  = util.patch.pixel_coords_to_patch(mask_xy,image_h,image_w,ws,ws)
   lookup_xy = util.patch.pixel_coords_to_patch(mask_xy,image_h,image_w,lookup_h,lookup_w)

   local t1 = timer:time()
   local s1 = t1.real - t0.real
   local s2 = 0 
   local s3 = 0
   local s4 = 0
   local s5 = 0
   local s6 = 0
   log.tic()
   for pi = 1,mask_offsets:size(1) do

      collectgarbage()
      
      local t2 = timer:time()
      local mask_h  = mask_xy[1][pi]
      local mask_w  = mask_xy[2][pi] 

      local patch_h = patch_xy[1][pi]
      local patch_w = patch_xy[2][pi]

      -- find window in which to lookup patches
      local lookup_h = lookup_xy[1][pi]
      local lookup_w = lookup_xy[2][pi]

      if debug then
         printf("Processing patch: %d %d", patch_h,patch_w)
      end

      -- extract patch
      local patch = pano_patches[{{},patch_h,patch_w,{},{}}]:clone()
      _G.gpatch = patch
      -- prepare mask patch we are copying into (gets updated as we go)
      local patch_mask = tofill_patches[{patch_h,patch_w,{},{}}]:clone()
      patch_mask:cmul(image.gaussian(ws))      
      
      if debug then
         printf("Loading %d,%d lookup table of patches",lookup_h,lookup_w)
      end

      local lookup = pano_lookup[{{},lookup_h,lookup_w,{},{}}]:unfold(2,ws,1):unfold(3,ws,1)
      _G.glookup = pano_lookup[{{},lookup_h,lookup_w,{},{}}]
      -- mask of lookup we are coping from (only original image)
      local lookup_mask_patches = lookup_mask_windows[{lookup_h,lookup_w,{},{}}]:unfold(1,ws,1):unfold(2,ws,1)

      local t3 = timer:time()
      s2 = s2 +  t3.real - t2.real

      -- TODO: make util.patch.get_centers handle the 3 channels
      local lookup_centers1 = util.patch.get_centers(pano_lookup[{1,lookup_h,lookup_w,{},{}}],ws,ws)
      local lookup_centers2 = util.patch.get_centers(pano_lookup[{2,lookup_h,lookup_w,{},{}}],ws,ws)
      local lookup_centers3 = util.patch.get_centers(pano_lookup[{3,lookup_h,lookup_w,{},{}}],ws,ws)

      local lookup_mask_centers = util.patch.get_centers(lookup_mask_windows[{lookup_h,lookup_w,{},{}}],ws,ws):clone()
      lookup_mask_centers[lookup_mask_centers:lt(1)] = ws * ws

      local lookup_stride = lookup:stride()
      local n_lookup_h    = lookup:size(2)
      local n_lookup_w    = lookup:size(3)
      local n_lookup_tot  = n_lookup_h * n_lookup_w

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
         local row_mask = torch.cmul(patch_mask,lookup_mask_patches[c])

         -- compute L1 distance (so much faster than squaring)
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
      local patch_offset    = rand_index[dsti:narrow(1,1,n_found)]
      local lookup_patch_xy = util.addr.offset_to_pixel_coords(patch_offset,lookup_stride)

      -- pick random patch from top n
      local ri = math.random(n_found)

      if false then
         -- display patch + n_random_patches best matches
         local dpatches = torch.Tensor(n_found+1,3,ws,ws):zero()
         dpatches[1]:copy(patch)
         for i = 1,n_found do 
            dpatches[n_found]:copy(lookup[{{},lookup_patch_xy[{1,i}],lookup_patch_xy[{2,i}],{},{}}])
         end
         _G.gdpatches = dpatches
         -- image.display{image=dpatches,zoom=5,nrow=31}
      end

      lookup_patch_xy = lookup_patch_xy:narrow(2,ri,1)

      local pixel_in_lookup = util.patch.pixel_coords_patch_to_image(lookup_patch_xy,ws,ws,n_lookup_h,n_lookup_w)

      lookup_patch_xy = lookup_patch_xy:squeeze()

      local pixel_in_image = util.patch.patch_pixel_coords_to_image_pixel_coords(pixel_in_lookup,lookup_h,lookup_w)
      pixel_in_image = pixel_in_image:squeeze()

      printf("img: %d %d patch: %d %d", mask_h, mask_w, pixel_in_image[1],pixel_in_image[2])

      -- copy pixel data
      img_rgb[{{},mask_h,mask_w}]  =  img_rgb[{{},pixel_in_image[1],pixel_in_image[2]}]
      pano_lab[{{},mask_h,mask_w}] = pano_lab[{{},pixel_in_image[1],pixel_in_image[2]}]

      -- print(img_rgb[{{},mask_h,mask_w}])
      if (img_rgb[{{},mask_h,mask_w}]:sum() == 0) then 
         px = img_rgb[{{},mask_h,mask_w}]
         printf(" copied: %f %f %f",px[1],px[2],px[3])
         printf(" center: %f %f %f",
             lookup_centers1[lookup_patch_xy[1]][lookup_patch_xy[2]],
             lookup_centers2[lookup_patch_xy[1]][lookup_patch_xy[2]],
             lookup_centers3[lookup_patch_xy[1]][lookup_patch_xy[2]])
         print("** COPIED zero value **")
         print(mask_patches[lookup_patch_xy[1]][lookup_patch_xy[2]])
         printf("index: %d",ri)
         print(distances[ri])
      end

      -- update mask (zeros in alpha channel need to be filled)
      global_tofill_mask[mask_h][mask_w] = 1
      local t7 = timer:time()
      s6 = s6 + t7.real - t6.real
   end
   printf(" - processed %d patches in %2.4fs", n_patches, log.toc())
   print("time:")
   printf(" - setup                %2.4fs",s1)
   print("in loop:")
   printf(" - extract data         %2.4fs",s2)
   printf(" - get_centers, expand  %2.4fs",s3)
   printf(" - compute distances    %2.4fs",s4)
   printf(" - sort                 %2.4fs",s5)
   printf(" - apply                %2.4fs",s6)
end

mask_offsets = find_patches(global_tofill_mask,ws)

count=1
-- for i = 1,1 do 
while mask_offsets do 
   fill_patches(mask_offsets,global_lookup_mask_windows)
   mask_offsets = find_patches(global_tofill_mask,ws)
   collectgarbage() 
   if (count % 10 == 0) then
      image.save(out_file,img_rgb)
   end
   count = count + 1
end

image.save(out_file,img_rgb)

_G.gimg_rgb = img_rgb
_G.gtofill  = global_tofill_mask
_G.gmask    = global_lookup_mask
