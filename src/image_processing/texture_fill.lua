require 'image'

batch_size = 100
ws = 3
ctr = math.ceil(ws*0.5)
half = ws * ws * 0.5 


image_plus_alpha = image.load('full-360-masked-hdr-downsampled-small.png')

mask = image_plus_alpha[4]
mask[mask:gt(0)] = 1 -- binarize

pano_rgb = image_plus_alpha:narrow(1,1,3)
image_h  = pano_rgb:size(2)
image_w  = pano_rgb:size(3)
pano_lab = image.rgb2lab(pano_rgb)


mask_patches = mask:unfold(1,ws,1):unfold(2,ws,1)

mps = mask_patches:size()

pano_patches = pano_lab:unfold(2,ws,1):unfold(3,ws,1)
-- flatten
npatches = mps[1]*mps[2]
pano_patches = pano_patches:reshape(3,npatches,ws,ws)
mask_patches = mask_patches:reshape(npatches,ws,ws)

-- make list of patches with sum > 0 and center pixel == 0

-- operation is sum of all windows (which we can easily make window
-- apply function for)

sys.tic()
-- dilation kernel
kernel = torch.ones(ws,ws)
-- center of match must be zero so set center kernel to -(number_of_offcenter_pixels)
kernel[ctr][ctr] = -((ws * ws) - 1)
-- apply dilation which is fast even on large input
mask_dilated = torch.conv2(mask,kernel,'F'):narrow(1,ctr-1,image_h):narrow(2,ctr-1,image_w)
mask_dilated[mask_dilated:lt(0)] = 0

n_neighbors = ws * (ctr-1)
byte_index = mask_dilated:gt(n_neighbors)
found = byte_index:sum() 

while ((found == 0) and (n_neighbors > 0)) do
   n_neighbors = n_neighbors - 1
   byte_index = mask_dilated:gt(n_neighbors)
   found = byte_index:sum() 
end 

if found == 0 then 
   print("Filled all patches: quit")
end


mask_range = torch.range(1,image_h*image_w):reshape(image_h,image_w)
mask_index = mask_range[byte_index] 


-- sort by most neighbors
y,i = mask_dilated[byte_index]:sort(true)

if i:size(1) > batch_size then 
   i = i:narrow(1,1,batch_size)
end

mask_index = mask_index[i]

printf(" - found %d > %d in %2.4fs",found, n_neighbors, sys.toc())

-- sys.tic()
-- windows_todo = torch.LongTensor(npatches)
-- found = 0
-- for i = 1,npatches do 
--    local s = mask_patches[i]:sum()
--    local c = mask_patches[i][ctr][ctr]
--    if ((s > ws)  and (c == 0)) then 
--       found = found + 1
--       windows_todo[found] = i
--    end
-- end

-- windows_todo = windows_todo:narrow(1,1,found)

-- printf("Found %d windows todo in %2.4fs",found, sys.toc())

-- for each todo patch compute distance with all other patches

mask_todo = mask_patches[mask_index]  

