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

poses = Poses.new(posefile)

target = util.obj.load(targetfile,10)

-- modify this to load per Sweep

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



