Class()

pi = math.pi
pi2 = pi * 0.5

width = 4000
height = 3000

hfov = pi2
vfov = pi2 * height / width
 
rp = projection.RectilinearProjection.new(width,height, hfov,vfov)

angles = rp:angles_map()

perElement = 1/angles[1]:nElement()

p("Testing Rectilinear projection")
log.tic()
unit_coords = rp:angles_to_normalized_coords(angles)
time = log.toc()
printf(" - angles to coords %2.4fs, %2.4es per px", time, time*perElement)
log.tic()
unit_angles = rp:normalized_coords_to_angles(unit_coords)
time = log.toc()
printf(" - normalized_coords to angles %2.4fs, %2.4es per px", time, time*perElement)

err = unit_angles - angles
err:abs()
printf(" - unit -> angle error (> 1e-15) : %d/%d , mean: %f", err:gt(1e-15):sum(), err[1]:nElement(), err:mean())

log.tic()
pixels = rp:angles_to_pixels(angles)
time = log.toc()
printf(" - angles to pixels %2.4fs %2.4es per px", time, time*perElement)
log.tic()
pixels_angles = rp:pixels_to_angles(pixels)
time = log.toc()
printf(" - pixels to angles %2.4fs, %2.4es per px", time, time*perElement)

err = pixels_angles - angles
err:abs()
printf(" - pixels -> angle error (> 1e-15) : %d/%d , mean: %f", err:gt(1e-15):sum(), err[1]:nElement(), err:mean())


