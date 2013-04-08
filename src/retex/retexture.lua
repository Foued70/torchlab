cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Make textures')
cmd:text()
cmd:text('Options')
cmd:option('-targetfile',
            "/Users/judyhe/Downloads/withered-dust-2012_a_03/rivercourt_3307_v3.obj",
           'target obj with new geometry')
cmd:option('-posefile',
           '/Users/judyhe/Downloads/withered-dust-2012_a_00/scanner371_job224000_texture_info.txt',
           'pose info file in same directory as the texture images')
cmd:option('-scale',4,'scale at which occlusions where processed')
-- cmd:option('-maskdir','texture_swap/mask/','mask for retexture')
cmd:text()

-- parse input params
params = cmd:parse(arg)

local scan = util.mp.scan(params.posefile, params.targetfile)
local tex = retex.Textures.new(scan, {scale = params.scale})
tex:make()