local pi = math.pi
local pi2 = pi * 0.5

local width  = 4000
local height = 3000

local hfov = pi2 
local vfov = hfov * height / width
 
local gp = projection.GnomonicProjection.new(width,height, hfov,vfov)
local rp = projection.RectilinearProjection.new(width,height, hfov,vfov)

local angles = gp:angles_map()
local angles_rp = rp:angles_map()

local unit_coords, unit_coords_rp
local pixels, pixels_rp, pixels_angles
local time, err

perElement = 1/angles[1]:nElement()

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
printf(" - Error Gnomonic unit -> angle (self) error (> 1e-15)")
printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())

err = angles - angles_rp
err:abs()
printf(" - Error angles Gnomonic vs. Rect (> 1e-15)")
printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())

err = unit_coords - unit_coords_rp
err:abs()
printf(" - Error unit Gnomonic vs. Rect (> 1e-15)")
printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())

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
printf(" - Error Gnomonic pixels -> angle (self) error (> 1e-15)")
printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())

err = pixels - pixels_rp
err:abs()
printf(" - Error pixels Gnomonic vs Rect. error (> 1e-11)")
printf("   %d/%d , max: %e", err:gt(1e-11):sum(), err[1]:nElement(), err:max())

angles_rp = nil
pixels_rp = nil
unit_coords_rp = nil
collectgarbage()

centers = {{pi2,0},{pi,0},{-pi2,0},{0,0},{0,pi2},{0,-pi2}}

for i,c in ipairs(centers) do 
   printf("Testing Gnomonic projection off axis (%f,%f)", c[1],c[2])
   gp:set_lambda_phi(c[1],c[2])
   angles = gp:angles_map()
   sys.tic()
   unit_coords = gp:angles_to_coords(angles)
   time = sys.toc()
   printf(" - Time Gnomonic angles to coords")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)
   sys.tic()
   unit_angles = gp:coords_to_angles(unit_coords)
   time = sys.toc()
   printf(" - Time Gnomonic coords to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = unit_angles - angles
   err:abs()
   printf(" - Error Gnomonic unit -> angle (self) error (> 1e-15)")
   printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())


   sys.tic()
   pixels = gp:angles_to_pixels(angles)
   time = sys.toc()
   printf(" - Time Gnomonic angles to pixels")
   printf("   %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()
   pixels_angles = gp:pixels_to_angles(pixels)
   time = sys.toc()
   printf(" - Time Gnomonic pixels to angles")
   printf("   %2.4fs, %2.4es per px", time, time*perElement)

   err = pixels_angles - angles
   err:abs()
   printf(" - Error Gnomonic pixels -> angle (self) error (> 1e-15)")
   printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())
   
   angles = nil
   unit_coords = nil
   unit_angles = nil
   pixels = nil
   pixels_angles = nil
   err = nil
   collectgarbage()
end

