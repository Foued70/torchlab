local Occlusions = require('Occlusions')

-- top level filenames
cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()

cmd:option('-targetfile',
           "/Users/judyhe/Downloads/withered-dust-2012_a_06/bare-model.obj",
           --'test/invincible-violet/retexture-tworoom.obj',
           'target obj with new geometry')
cmd:option('-posefile',
          "/Users/judyhe/Downloads/withered-dust-2012_a_00/scanner371_job224000_texture_info.txt",
           -- 'models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt',
           --'models/invincible-violet-3396_a_00/scanner371_job129001_texture_info.txt',
           'pose info file in same directory as the texture images')
cmd:option('-scale',4,'scale at which to process 4 = 1/4 resolution')
cmd:option('-packetsize',0,'window size for ray packets (32x32)')
cmd:text()


-- parse input params
local params = cmd:parse(arg)

local occlusions = Occlusions.new(params.posefile, params.targetfile, params.scale, params.packetsize)
occlusions:calc(true)
