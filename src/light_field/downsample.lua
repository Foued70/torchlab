epi_file = 'rail_data/tone_mapped_512x339/output/epi_image_000001-000339_torch.FloatTensor.t7'

epi = torch.load(epi_file)
nchan  = epi:size(1)
nimg   = epi:size(2)
height = epi:size(3)
width  = epi:size(4)

function display_images(epi,nrow)
   nrow = nrow or 4
   imgs = {}
   for i = 1,epi:size(2) do 
      table.insert(imgs,epi[{{},i,{},{}}])
   end
   image.display{image=imgs,nrow=nrow}
end

display_images(epi)

epis = {}
epi_previous = epi:reshape(nchan*nimg,height,width)
while (height > 7) do 
   
   height = math.ceil(height * 0.5)
   width  = math.ceil(width * 0.5)

   log.trace("making ".. height .. " x " .. width)

   epi_current = torch.FloatTensor(nchan*nimg,height,width)
   epi_previous.image.scaleSimple(epi_previous,epi_current)

   table.insert(epis,epi_current:reshape(nchan,nimg,height,width))

   epi_previous = epi_current

end

for _,img in pairs(epis) do 
   display_images(img)
end
