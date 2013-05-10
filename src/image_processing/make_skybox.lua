local projection_util = util.projection
require 'image'

sys.tic()

pi = math.pi
pi2 = pi * 0.5
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
out_fov  = pi2

proj_from = projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)

p("Creating Skybox Projection")
time_prep = sys.toc()
printf(" - load image in %2.4fs", time_prep)
sys.tic()

-- make a skybox
proj_to = {}
angle_maps  = {}
index_and_masks = {}
centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}
-- this naming comes from unity and is from the outside looking in
names   = {"front", "left", "back", "right", "down", "up"}
for _,off in ipairs(centers) do 

   proj   = projection.GnomonicProjection.new(out_size,out_size,
                                              out_fov,out_fov,
                                              out_size/2,out_size/2,
                                              off[1],off[2])
   table.insert(proj_to,proj)
   angle_map = proj:angles_map()
   table.insert(angle_maps, angle_map)

   local angles = angle_map:clone()
   local x = angles[1]
   local y = angles[2]

   local sign = torch.sign(angles)
   angles:abs()
   
   yover = angles[2]:gt(pi2)
   n_yover = yover:sum()
   if (n_yover > 0) then 
      printf(" - fixing %d pixels", n_yover)
      -- come back down as much as you went over
      y[yover] = y[yover]:mul(-1):add(pi)
      -- spin x around to the other side of the globe
      x[yover] = x[yover]:add(pi)
   end
   xover = angles[1]:gt(pi)
   n_xover = xover:sum()
   if (n_xover > 0) then
      printf(" - fixing %d pixels", n_xover)
      x[xover] = x[xover]:add(-2*pi)
   end

   angles:cmul(sign)
   table.insert(index_and_masks,
                proj_from:angles_to_index1D_and_mask(angles))
end

time_map = sys.toc()
printf(" - make map %2.4fs", time_map)
sys.tic()

for i,idx in ipairs(index_and_masks) do 
   local face = projection_util.remap(img,idx)
   local outf = out_file .."_"..names[i]..".jpg"
   image.display(face)
   printf(" - saving: %s", outf)
   image.save(outf,face)
end

time_reproject = sys.toc()
printf(" - reproject %2.4fs", time_reproject)
sys.tic()


printf(" - Total %2.4fs", time_prep + time_map + time_reproject)
