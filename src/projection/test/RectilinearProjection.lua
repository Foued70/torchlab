local pi = math.pi
local pi2 = pi * 0.5

local width = 4000
local height = 3000

local hfov = pi2
local vfov = pi2 * height / width
 
local rp = projection.RectilinearProjection.new(width,height, hfov,vfov)

local angles = rp:angles_map()

local perElement = 1/angles[1]:nElement()

p("Testing Rectilinear projection")
sys.tic()
local unit_coords = rp.angles_to_coords(angles)
time = sys.toc()
printf(" - angles to coords %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
local unit_angles = rp.coords_to_angles(unit_coords)
time = sys.toc()
printf(" - coords to angles %2.4fs, %2.4es per px", time, time*perElement)

local err = unit_angles - angles
err:abs()
printf(" - unit -> angle error (> 1e-15) : %d/%d , mean: %f", err:gt(1e-15):sum(), err[1]:nElement(), err:mean())

sys.tic()
local pixels = rp:angles_to_pixels(angles)
time = sys.toc()
printf(" - angles to pixels %2.4fs %2.4es per px", time, time*perElement)
sys.tic()
local pixels_angles = rp:pixels_to_angles(pixels)
time = sys.toc()
printf(" - pixels to angles %2.4fs, %2.4es per px", time, time*perElement)

local err = pixels_angles - angles
err:abs()
printf(" - pixels -> angle error (> 1e-15) : %d/%d , mean: %f", err:gt(1e-15):sum(), err[1]:nElement(), err:mean())


