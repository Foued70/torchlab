Class()

pi = math.pi
pi2 = pi * 0.5

width  = 400
height = 400

hfov = pi - 1e-3 -- stay away from nans
vfov = pi - 1e-3

gp = projection.StereographicProjection.new(width,height, hfov,vfov)

angles = gp:angles_map()

perElement = 1/angles[1]:nElement()

centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}

total     = 0
total_max = -math.huge
total_err = 0

function report_error(err,str,eps)
   eps = eps or 1e-16
   str = str or " - ERRORS "
   local cerr = err:gt(1e-13):sum()
   local ctot = err[1]:nElement()
   local cmax = err:max()
   printf(str)
   printf("   %d/%d , max: %e eps: %1.1e", cerr, ctot, cmax, eps)

   total = total + ctot
   total_err = total_err + cerr
   total_max = math.max(total_max, cmax)

   return cerr, ctot, cmax
end

p("Testing Stereographic projection")
sys.tic()

for i,c in ipairs(centers) do
   printf("Tangent point at (%f,%f)", c[1],c[2])
   gp:set_lambda_phi(c[1],c[2])
   angles = gp:angles_map()
   sys.tic()
   unit_coords = gp:angles_to_coords(angles,unit_coords)
   time = sys.toc()
   printf(" - Time Stereographic angles to coords")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)
   sys.tic()
   unit_angles = gp:coords_to_angles(unit_coords,unit_angles)
   time = sys.toc()
   printf(" - Time Stereographic coords to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = unit_angles - angles
   err:abs()

   report_error(err," - Error unit -> angle (self)",1e-13)

   -- test pixels

   sys.tic()
   pixels = gp:angles_to_pixels(angles,pixels)
   time = sys.toc()
   printf(" - Time Stereographic angles to pixels")
   printf("   %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   pixels_angles = gp:pixels_to_angles(pixels,pixels_angles)
   time = sys.toc()
   printf(" - Time Stereographic pixels to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = pixels_angles - angles
   err:abs()
   report_error(err," - Error pixels -> angle (self)",1e-13)

   collectgarbage()
end

printf("Total Errors:")
printf("   %d/%d , max: %e", total_err, total, total_max)