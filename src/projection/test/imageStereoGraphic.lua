Class()

require 'image'

pi = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagefile', 'equirectangular.jpg', 'Equirectangular input to remap')
cmd:option('-scale','0.25', 'downsample')
cmd:text()

-- parse input params
params = cmd:parse(arg)

image_file  = params.imagefile

if not paths.filep(image_file) then 
   error(string.format("-imagefile %s not found",image_file))
end

out_file = out_file or image_file:gsub("....$","")
img = image.load(image_file)

width = img:size(3)
height = img:size(2)
scale  = tonumber(params.scale)

hfov = 2 * pi 
vfov = pi
cx   = width * 0.5
cy   = height * 0.5

out_fov  = pi/1.05
out_size = scale * width
out_c    = out_size * 0.5


proj_sphere = projection.SphericalProjection.new(width,height,hfov,vfov,cx,cy)
-- little world
proj_stereo = projection.StereographicProjection.new(out_size,out_size,out_fov,out_fov,nil,nil,0,pi2)

p("Testing Image Projection")

sys.tic()

sphere_to_stereographic = projection.Remap.new(proj_sphere,proj_stereo)
-- do not need to call get_offset_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
offset = sphere_to_stereographic:get_offset_and_mask()
perElement = offset:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

img_out = sphere_to_stereographic:remap(img)
time = sys.toc()
printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

force = true
delta = torch.randn(2) * 0.1   
lam = pi2
phi = 0
while true do 
   lam = lam + delta[1]
   phi = phi + delta[2]
   proj_stereo:set_lambda_phi(lam,phi)
   img_out = sphere_to_stereographic:remap(img,force) 
   win = image.display{win=win,image={img_out}}
end
