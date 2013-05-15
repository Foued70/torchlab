Class()

require 'image'

sys.tic()

pi = math.pi

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute skybox projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagefile', '', '360x180 equirectangular image for skybox')
cmd:option('-outfile', 'skybox', 'basename for 6 output images for skybox')
cmd:option('-size', '1024', 'size in pixels of side of skybox cube')
cmd:text()

-- parse input params
params = cmd:parse(arg)

image_file  = params.imagefile
out_size   = params.size
out_file   = params.outfile

if not paths.filep(image_file) then 
   error(string.format("-imagefile %s not found",image_file))
end

out_file = out_file or image_file:gsub("....$","")
img = image.load(image_file)
img = img:narrow(1,1,3)

width      = img:size(3)
height     = img:size(2)

hfov = 2 * pi 
vfov = pi
cx   = width * 0.5
cy   = height * 0.5

proj_from = projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)

time_prep = sys.toc()
printf(" - load image in %2.4fs", time_prep)
sys.tic()

p("Creating Skybox Projection")
cmap = projection.CubeMap.new(proj_from,out_size)

time_map = sys.toc()
printf(" - make map %2.4fs", time_map)
sys.tic()

faces = cmap:remap(img)
for name,face in pairs(faces) do 
   outf = out_file .."_"..name..".jpg"
   image.display(face)
   printf(" - saving: %s", outf)
   image.save(outf,face)
end

time_reproject = sys.toc()
printf(" - reproject %2.4fs", time_reproject)
sys.tic()

printf(" - Total %2.4fs", time_prep + time_map + time_reproject)
