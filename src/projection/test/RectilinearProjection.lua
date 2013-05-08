local pi = math.pi

local rp = projection.RectilinearProjection.new(800,600, pi/2)

local angles = torch.Tensor({
  {0,0},
  {0,pi/2},
  {0,pi/4},
  {0,-pi/2},
  {0,-pi/4},
  {pi/2,0},
  {pi/4,0},
  {-pi/2,0},
  {-pi/4,0},
  {pi/4,pi/4}
})

p("angles:")
p(angles)

local angles = angles:t():contiguous()

local unit_coords = rp:angles_to_coords(angles)
p("unit coords:")
p(unit_coords:t())
p("coords to angles:")
p(rp:coords_to_angles(unit_coords):t())

local pixels = rp:angles_to_pixels(angles)
p("pixels:")
p(pixels:t())
p("pixels to angles:")
p(rp:pixels_to_angles(pixels):t())
