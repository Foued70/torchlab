local width  = 4000
local height = 3000

 
local rp = projection.Projection.new(width,height)

pixels = rp:pixels_map()

local perElement = 1/pixels[1]:nElement()

p("Testing Rectilinear projection")
sys.tic()
index1D_and_mask = rp:pixels_to_index1D_and_mask(pixels)
time = sys.toc()
printf(" - pixels to index1D_and_mask %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
pixels_out = rp:index1D_and_mask_to_pixels(index1D_and_mask)
time = sys.toc()
printf(" - index1D_and_mask to pixels %2.4fs, %2.4es per px", time, time*perElement)

local err = pixels_out - pixels
err:abs()
printf(" - pixels -> index1D_and_mask : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())
