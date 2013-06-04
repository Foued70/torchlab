Class()

width  = 2000
height = 1500

rp = projection.Projection.new(width,height)

pixels = rp:pixels_map()

perElement = 1/pixels[1]:nElement()

p("Testing indexing of projections: pixels")
sys.tic()
offset,stride,mask = rp:pixels_to_offset_and_mask(pixels)
time = sys.toc()
printf(" - pixels to offset_and_mask %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
pixels_out = rp:offset_and_mask_to_pixels(offset,stride,mask)
time = sys.toc()
printf(" - offset_and_mask to pixels %2.4fs, %2.4es per px", time, time*perElement)

err = pixels_out - pixels
err:abs()
printf(" - pixels -> offset_and_mask : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())

rp = projection.RectilinearProjection.new(width,height,math.pi*0.45)
angles = rp:angles_map()

p("Testing indexing of projections: angles")
sys.tic()
offset,stride,mask = rp:angles_to_offset_and_mask(angles)
time = sys.toc()
printf(" - angles to offset_and_mask %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
angles_out = rp:offset_and_mask_to_angles(offset,stride,mask)
time = sys.toc()
printf(" - offset_and_mask to angles %2.4fs, %2.4es per px", time, time*perElement)

err = angles_out - angles
err:abs()
printf(" - angles -> offset_and_mask : %d/%d , mean: %f", err:gt(0):sum(), err[1]:nElement(), err:mean())
