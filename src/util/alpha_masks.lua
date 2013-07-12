Class()
-- utilities for masks and alpha channels

function inverse(alpha)
   return alpha:add(-alpha:max()):abs()
end

-- takes a table of aligned 4 channel RGBA images or
-- 2 tables of aligned images and an equal number of alpha channels and blends.
-- and alpha channel is between 0 and 1

function blend (images, alpha, invert)
   local all_alpha = torch.zeros(images[1][1]:size())
   if alpha then
      -- separate table of alpha channels
      if alpha[1]:type() == 'torch.DoubleTensor' then
         for i = 1,#alpha do
            all_alpha:add(alpha[i])
         end
      else
         for i = 1,#alpha do
            all_alpha:add(alpha[i]:double())
         end
      end
   elseif (images[1]:size(1) == 4) then
      -- RGBA images
      for i = 1,#alpha do
         all_alpha:add(images[i][4])
      end
   else
      error("images are either 4 channel (eg. RGBA) or pass a second table of alpha channels")
   end

   -- make minumum == 0
   all_alpha:add(-(all_alpha:min()-1))
   -- invert
   if invert then inverse(all_alpha) end
   -- sum to 1
   all_alpha = torch.cdiv(torch.ones(all_alpha:size()),all_alpha)

   -- out of bounds
   all_alpha[all_alpha:eq(math.huge)] = 0

   all_alpha_size = all_alpha:size()
   all_alpha:resize(util.util.add_slices(1,all_alpha_size))
   all_alpha = all_alpha:expand(util.util.add_slices(3,all_alpha_size))

   all_images = torch.cmul(images[1],all_alpha)
   for i = 2,#images do
      all_images:add(torch.cmul(images[i],all_alpha))
   end
   return all_images, all_alpha
end
