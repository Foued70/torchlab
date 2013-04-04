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

local tex = retex.Textures.new(params.posefile, params.targetfile, {scale = params.scale})
tex:make()

-- for fid = 1, tex.target.n_faces do
--   tex:create_uvs(fid)
--   local material = {
--     name = "face"..fid,
--     diffuse = {0.5, 0.5, 0.5, 1},
--     diffuse_tex_path = paths.basename(tex:file(fid))    
--   }
--   table.insert(tex.target.materials, material)
--   table.insert(tex.target.submeshes, {fid, fid, #tex.target.materials})
-- end
-- tex:save_obj()
