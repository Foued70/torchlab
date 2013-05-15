Class()

width  = 4000
height = 3000

rp = projection.Projection.new(width,height)

pixels = rp:pixels_map()

perElement = 1/pixels[1]:nElement()

p("Testing indexing of projections: pixels")
sys.tic()
index1D,stride,mask = rp:pixels_to_index1D_and_mask(pixels)
time = sys.toc()
printf(" - pixels to index1D_and_mask %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
pixels_out = rp:index1D_and_mask_to_pixels(index1D,stride,mask)
time = sys.toc()
printf(" - index1D_and_mask to pixels %2.4fs, %2.4es per px", time, time*perElement)

err = pixels_out - pixels
err:abs()
printf(" - pixels -> index1D_and_mask : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())

rp = projection.RectilinearProjection.new(width,height,math.pi*0.45)
angles = rp:angles_map()

p("Testing indexing of projections: angles")
sys.tic()
index1D,stride,mask = rp:angles_to_index1D_and_mask(angles)
time = sys.toc()
printf(" - angles to index1D_and_mask %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
angles_out = rp:index1D_and_mask_to_angles(index1D,stride,mask)
time = sys.toc()
printf(" - index1D_and_mask to angles %2.4fs, %2.4es per px", time, time*perElement)

err = angles_out - angles
err:abs()
printf(" - angles -> index1D_and_mask : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())
