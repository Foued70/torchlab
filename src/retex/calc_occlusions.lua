-- top level filenames
cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()

cmd:option('-targetfile',
           "/Users/judyhe/Downloads/jet-wildflower-1466_a_03/export-aligned.obj",
           --'test/invincible-violet/retexture-tworoom.obj',
           'target obj with new geometry')
cmd:option('-posefile',
          "/Users/judyhe/Downloads/jet-wildflower-1466_a_00/scanner621_job215001_texture_info.txt",
           -- 'models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt',
           --'models/invincible-violet-3396_a_00/scanner371_job129001_texture_info.txt',
           'pose info file in same directory as the texture images')
cmd:option('-scale',16,'scale at which to process 4 = 1/4 resolution')
cmd:option('-packetsize',0,'window size for ray packets (32x32)')
cmd:text()


-- parse input params
local params = cmd:parse(arg)

-- mp scan
--local scan = util.mp.scan(params.posefile, params.targetfile)
--params.scale = 4

-- floored scan
local scan = util.Scan.new('/Users/judyhe/Downloads/96_spring_kitchen')
scan:get_depth_maps(params.scale, params.packetsize)