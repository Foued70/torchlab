Class()
-- 
-- image patch helpers
-- 
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

function get_centers(mask,patch_height,patch_width)

   local mask_height = mask:size(1)
   local mask_width  = mask:size(2)

   local ctr_height = math.ceil(patch_height * 0.5)
   local ctr_width  = math.ceil(patch_width * 0.5)
   local n_height   = mask_height - patch_height + 1
   local n_width    = mask_width  - patch_width + 1

   return mask:narrow(1,ctr_height,n_height):narrow(2,ctr_width,n_width)
end
