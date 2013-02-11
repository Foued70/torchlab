require 'image'

local util = require 'util'
local geom = util.geom

local r2d = 180 / math.pi
local d2r = math.pi / 180

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
cmd:text('Options')
cmd:option('-targetfile',
           --           "models/rivercourt_3307_regeom/rivercourt_3307.obj",
           "../data/models/withered-dust-2012_a_03/rivercourt_3307_v3.obj",
           'target obj with new geometry')
cmd:option('-posefile',
           '../data/test/texture_swap/scanner371_job224000_texture_info.txt',
--           "models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt",
           'pose info file in same directory as the texture images')
cmd:option('-imagesdir',
           'images/',
           'directory with the images to load')
cmd:option('-outdir','output/')
cmd:text()

-- parse input params
params = cmd:parse(arg)

targetfile = params.targetfile
posefile   = params.posefile
imagesdir  = params.imagesdir
outdir     = params.outdir .. "/"

cachedir = "../cache/"

sys.execute("mkdir -p " .. cachedir)
sys.execute("mkdir -p " .. outdir)

posecache   = cachedir ..   posefile:gsub("/","_"):gsub("%.","dot") .. ".t7"
targetcache = cachedir .. targetfile:gsub("/","_"):gsub("%.","dot") .. ".t7"
posedir     = paths.dirname(posefile)

-- 
-- NOTES: 
--
-- + We need somekind of mapping from matterport poses to a set of
-- images in a directory.  Unfortunately I didn't start the
-- photographic sweep in the same orientation as the matterport sweep,
-- so we have an initial rotation to discover.
--

-- FIXME should really make this reusable rather than copy into every executable
function loadcache (objfile,cachefile,loader,args)
   local object = nil
   -- Process or load the poses
   if (paths.filep(cachefile)) then
      sys.tic()
      object = torch.load(cachefile)
      printf("Loaded %s from %s in %2.2fs", objfile, cachefile, sys.toc())
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      printf("Saving %s to %s", objfile, cachefile)
   end
   return object
end

-- load pose file
if not poses then
   poses  = loadcache(posefile,posecache,Poses.new)
end

-- load model
if not target then
   target = loadcache(targetfile,targetcache,util.obj.load,10)
end

-- load images
if not images then
   images = {}
   imgfiles = paths.files(imagesdir)
   imgfiles()
   imgfiles()
   local cnt = 1
   for f in imgfiles do 
      local imgfile = imagesdir.."/"..f
      printf("Loading : %s", imgfile)
      local img = image.load(imgfile)
      images[cnt] = image.vflip(img:transpose(2,3))
      cnt = cnt + 1
   end
end

-- start of camera object (extension of pose object...)
-- write .ini loader...
camera = {
   name    = "nikon_d800E_w18mm",
   sensor_w = 35.9, -- mm
   sensor_h = 24.0, -- mm
   focal   = 18,   -- mm
   image_w    = images[1]:size(3), -- px
   image_h    = images[1]:size(2), -- px
   -- copied from .ini computed in hugin
   a       =  0.0762999,
   b       = -0.167213,
   c       =  0.061329,
   -- contains lens parameters to rectify the images on the sensor to
   -- equal rays in 3D space, the extrinsic (pose) parameters then
   -- rotate and translate the rays to the correct position and
   -- orientation in the world.
   intrinsic = torch.eye(3)
}

-- same as in pose
camera.center_x = camera.image_w*0.5
camera.center_y = camera.image_h*0.5
   
camera.fovw     = r2d * 2 * torch.atan((camera.sensor_w*0.5)/camera.focal) -- deg
camera.fovh     = r2d * 2 * torch.atan((camera.sensor_h*0.5)/camera.focal) -- deg

camera.degree_per_px_x = camera.fovw/camera.image_w
camera.degree_per_px_y = camera.fovh/camera.image_h

-- focal length in pixels
camera.focal_px_x = (camera.center_x * camera.focal)/camera.sensor_w * 0.5
camera.focal_px_y = (camera.center_y * camera.focal)/camera.sensor_h * 0.5

-- intrinsic matrix is stored in pixels as we will lookup in pixels.
camera.intrinsic[1][1] = camera.focal_px_x
camera.intrinsic[2][2] = camera.focal_px_y
camera.intrinsic[1][3] = camera.center_x
camera.intrinsic[2][3] = camera.center_y

-- pose = poses[1] 
-- mptexture = pose.image

-- newtexture = torch.zeros(mptexture:size())
-- texw = newtexture:size(3)
-- texh = newtexture:size(2)


-- posecam = {
--    name = pose.name
--    imgw = pose.w
--    imgh = pose.h
--    fovh = pose.h * pose.px[2]
--    fovw = pose.w * pose.px[1]
--    intrinsic = torch.eye(3)
-- }

-- posecam.intrinsic[1][1] = pose.px[2]
-- posecam.intrinsic[2][2] = pose.px[1]



-- scale = texh / camera.imgh
-- -- generate xymap
-- maph = 10 -- math.floor(imgw*scale + 0.5 )
-- mapw = 10 -- math.floor(imgh*scale + 0.5 )

-- row = torch.linspace(5,camera.imgw-5,mapw)
-- col = torch.linspace(5,camera.imgh-5,maph)

-- map = torch.ones(maph,mapw,3)

-- -- fill cols 
-- for c = 1,mapw do 
--    map[{{},c,1}] = col
-- end

-- for r = 1,maph do 
--    map[{r,{},2}] = row
-- end

-- -- flatten
-- map:resize(maph*mapw,3)

-- -- do subpixel later
-- map:add(0.5):floor()

-- for i = 1,map:size(1) do 
--    local imap = map[i]
--    local tmap = torch.mv(camera.intrinsic,imap)
--    printf("(%d, %d) -> (%f, %f, %f)",imap[1],imap[2],tmap[1],tmap[2],tmap[3])
--    -- newtexture[1][tmap[1]][tmap[2]] = image[1][1][imap[1]][imap[2]]
--    -- newtexture[2][tmap[1]][tmap[2]] = image[1][2][imap[1]][imap[2]]
--    -- newtexture[3][tmap[1]][tmap[2]] = image[1][3][imap[1]][imap[2]]
-- end

-- image.display{image={mptexture,newtexture},nrow=1}