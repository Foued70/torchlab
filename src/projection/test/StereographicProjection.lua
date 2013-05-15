local pi = math.pi
local pi2 = pi * 0.5

local width  = 4000
local height = 3000

local hfov = pi2
local vfov = hfov * height / width

local gp = projection.StereographicProjection.new(width,height, hfov,vfov)

local angles = gp:angles_map()

local unit_coords, unit_coords_rp
local pixels, pixels_rp, pixels_angles
local time, err

perElement = 1/angles[1]:nElement()

centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}

for i,c in ipairs(centers) do
   printf("Testing Stereographic projection off axis (%f,%f)", c[1],c[2])
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
   printf(" - Error Stereographic unit -> angle (self) error (> 1e-15)")
   printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())


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
   printf(" - Error Stereographic pixels -> angle (self) error (> 1e-15)")
   printf("   %d/%d , max: %e", err:gt(1e-15):sum(), err[1]:nElement(), err:max())

   angles = nil
   err = nil
   collectgarbage()
end

