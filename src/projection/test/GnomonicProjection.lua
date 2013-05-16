Class()

pi = math.pi
pi2 = pi * 0.5

width  = 400
height = 400

hfov = pi2 - 1e-3
vfov = pi2 - 1e-3

gp = projection.GnomonicProjection.new(width,height, hfov,vfov)
rp = projection.RectilinearProjection.new(width,height, hfov,vfov)

angles    = gp:angles_map()
angles_rp = rp:angles_map()

perElement = 1/angles[1]:nElement()

total     = 0
total_max = -math.huge
total_err = 0

function report_error(err,str,eps)
   eps = eps or 1e-16
   str = str or " - ERRORS "
   local cerr = err:gt(eps):sum()
   local ctot = err:nElement()
   local cmax = err:max()
   printf(str)
   printf("   %d/%d , max: %e eps: %1.1e", cerr, ctot, cmax, eps)

   total = total + ctot
   total_err = total_err + cerr
   total_max = math.max(total_max, cmax)

   return cerr, ctot, cmax
end

p("Testing Gnomonic projection")
sys.tic()
unit_coords = gp:angles_to_coords(angles)
time = sys.toc()
printf(" - Time Gnomonic angles to coords")
printf("   %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
unit_coords_rp = rp:angles_to_coords(angles_rp)
time = sys.toc()
printf(" - Time Rectilinear angles to coords")
printf("   %2.4fs, %2.4es per px", time, time*perElement)
sys.tic()
unit_angles = gp:coords_to_angles(unit_coords)
time = sys.toc()
printf(" - Time Gnomonic coords to angles")
printf("   %2.4fs, %2.4es per px", time, time*perElement)

err = unit_angles - angles
err:abs()

report_error(err," - Error unit -> angle",1e-15)

err = angles - angles_rp
err:abs()
report_error(err," - Error angles Gnomonic vs. Rect",1e-15)

err = unit_coords - unit_coords_rp
err:abs()
report_error(err," - Error unit Gnomonic vs. Rect",1e-15)

sys.tic()
pixels = gp:angles_to_pixels(angles)
time = sys.toc()
printf(" - Time Gnomonic angles to pixels")
printf("   %2.4fs %2.4es per px", time, time*perElement)
sys.tic()
pixels_rp = rp:angles_to_pixels(angles_rp)
time = sys.toc()
printf(" - Time Rectilinear angles to pixels")
printf("   %2.4fs %2.4es per px", time, time*perElement)
sys.tic()
pixels_angles = gp:pixels_to_angles(pixels)
time = sys.toc()
printf(" - Time Gnomonic pixels to angles")
printf("   %2.4fs, %2.4es per px", time, time*perElement)

err = pixels_angles - angles
err:abs()
report_error(err," - Error pixels -> angle",1e-15)

err = pixels - pixels_rp
err:abs()
report_error(err," - Error pixels Gnomonic vs Rect.",1e-11)

angles_rp = nil
pixels_rp = nil
unit_coords_rp = nil
collectgarbage()

centers = {{pi2,0},{pi,0},{-pi2,0},{0,0},{0,pi2},{0,-pi2}}
-- centers = {{pi,0},{0,pi2},{0,-pi2}}

for i,c in ipairs(centers) do
   printf("Testing tangent point(%f,%f)", c[1],c[2])
   gp:set_lambda_phi(c[1],c[2])
   angles = gp:angles_map()
   sys.tic()
   gp:angles_to_coords(angles,unit_coords)
   time = sys.toc()
   printf(" - Time Gnomonic angles to coords")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)
   sys.tic()
   gp:coords_to_angles(unit_coords,unit_angles)
   time = sys.toc()
   printf(" - Time Gnomonic coords to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = unit_angles - angles
   err:abs()
   report_error(err," - Error unit -> angle (self)",2e-13)
   
   sys.tic()
   gp:angles_to_pixels(angles,pixels)
   time = sys.toc()
   printf(" - Time Gnomonic angles to pixels")
   printf("   %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   gp:pixels_to_angles(pixels,pixels_angles)
   time = sys.toc()
   printf(" - Time Gnomonic pixels to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = pixels_angles - angles
   err:abs()
   report_error(err," - Error pixels -> angle (self)",2e-13)

   collectgarbage()
end

printf("Total Errors:")
printf("   %d/%d , max: %e", total_err, total, total_max)


