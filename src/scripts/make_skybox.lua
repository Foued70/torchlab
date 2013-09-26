-- Class()

log.tic()
pi = math.pi
d2r = pi / 180

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute skybox projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagefile', '', '360x180 equirectangular image for skybox')
cmd:option('-outfile', 'skybox', 'basename for 6 output images for skybox')
cmd:option('-size', 0, 'size in pixels of side of skybox cube')
cmd:option('-display', 0, 'display the cubemaps or not') 
cmd:option('-fov', 90 , 'field of view of each side of box')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

image_file = params.imagefile
out_size   = tonumber(params.size)
out_file   = params.outfile

fov = tonumber(params.fov)
if fov then 
   fov = d2r * fov
end

display_imgs = false
if params.display ~= 0 then 
   display_imgs = true
end

if not util.fs.is_file(image_file) then 
   error(string.format("-imagefile %s not found",image_file))
end

out_file = out_file or image_file:gsub("....$","")
img = image.load(image_file,'byte')
img = img:narrow(1,1,3)

width      = img:size(3)
height     = img:size(2)

if (out_size < 1) then 
   out_size = height/2
end

print(out_size)

hfov = 2 * pi 
vfov = pi
cx   = width * 0.5
cy   = height * 0.5

proj_from = projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)

time_prep = log.toc()
printf(" - load image in %2.4fms", time_prep)
log.tic()

p("Creating Skybox Projection")
cmap = projection.CubeMap.new(proj_from,out_size,fov)

time_map = log.toc()
printf(" - make map %2.4fms", time_map)
log.tic()

for fn = 1,6 do 
   face = cmap:remap(img,fn)
   name = cmap.names[fn]
 -- faces = cmap:remap(img)
-- for name,face in pairs(faces) do 
   outf = out_file .."_"..name..".png"
   printf(" - saving: %s", outf)
   image.save(outf,face)
   collectgarbage()
end

if display_imgs then
   image_tbl = {}
   for _,face in pairs(faces) do 
      table.insert(image_tbl, face)
   end
   image.display{image=image_tbl, nrow=3}
end

time_reproject = log.toc()
printf(" - reproject %2.4fms", time_reproject)
log.tic()

printf(" - Total %2.4fms", time_prep + time_map + time_reproject)
