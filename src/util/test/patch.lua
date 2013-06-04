-- Class()
ua = util.addr
up = util.patch

function test_pixel_coords_to_patch()
   local offset = torch.range(0,119)
   local xy     = ua.offset_to_pixel_coords(offset,torch.LongTensor({12,1}))

   for _,wsh in pairs({3,5,7,9}) do 
      for _,wsw in pairs({3,5,7,9}) do 
         printf("window: %d,%d", wsh,wsw)
         local patch_xy = up.pixel_coords_to_patch(xy,10,12,wsh,wsw,1)

         print(patch_xy[1]:resize(10,12))
         print(patch_xy[2]:resize(10,12))
      end
   end
end

function test_patch_pixel_coords_to_image_pixel_coords()
   local offset = torch.range(0,24)
   local xy    = ua.offset_to_pixel_coords(offset,torch.LongTensor({5,1}))

   local no_offset = up.patch_pixel_coords_to_image_pixel_coords(xy,1,1)
   print(no_offset[1]:resize(5,5))
   print(no_offset[2]:resize(5,5))
   local some_offset = up.patch_pixel_coords_to_image_pixel_coords(xy,10,100)
   print(some_offset[1]:resize(5,5))
   print(some_offset[2]:resize(5,5))
end


function test_get_centers ()
   local mask = torch.range(1,120):resize(10,12)
   
   for _,r in pairs({3,5,7,9}) do 
      print(mask)
      print(up.get_centers(mask,r,r))
   end
end

test_pixel_coords_to_patch()
test_patch_pixel_coords_to_image_pixel_coords()
test_get_centers()