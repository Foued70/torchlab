function blend (images, masks)

   allmask = masks[1]:clone()
   for i = 2,#masks do
      allmask:add(masks[i])
   end
   allmask:add(-(allmask:min()-1))

   allmask = allmask:double():add(-allmask:max()):abs()
   -- sum to 1
   allmask = torch.cdiv(torch.ones(allmask:size()),allmask)

   allmask[allmask:eq(math.huge)] = 0

   allmask_size = allmask:size()
   allmask:resize(util.util.add_slices(1,allmask_size))
   allmask = allmask:expand(util.util.add_slices(3,allmask_size))

   allimg = torch.cmul(images[1],allmask)
   for i = 2,#out_images do
      allimg:add(torch.cmul(images[i],allmask))
   end
   return allimg
end
