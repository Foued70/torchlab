local width  = 4000
local height = 3000

 
local rp = projection.Projection.new(width,height)

pixels = rp:pixels_map()

local perElement = 1/pixels[1]:nElement()

p("Testing Rectilinear projection")
sys.tic()
lookup = rp:pixels_to_lookup(pixels)
time = sys.toc()
printf(" - pixels to lookup %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
pixels_out = rp:lookup_to_pixels(lookup)
time = sys.toc()
printf(" - lookup to pixels %2.4fs, %2.4es per px", time, time*perElement)

local err = pixels_out - pixels
err:abs()
printf(" - pixels -> lookup : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())
