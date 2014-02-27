path = require 'path'
saliency = require '../image/saliency'
io = require 'io'

pi  = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Align a linear 360 sweep of images')
cmd:text()
cmd:text('Options')
cmd:option('-srcdir',      '/Users/lihui815/Documents/precise-transit-6548', 'top project directory')
cmd:option('-imdir',       'work/a_00/Images',                               'directory in which to find images')
cmd:option('-outdir',      'work/a_00/Aligned',                              'directory for aligned images')
cmd:option('-swdir',       '001',                                            'directory for specific sweep')
cmd:option('-imext',       '.tiff',                                          'image filename extensions')
cmd:option('-outname',     'panorama_360',                              'output image filename')
cmd:option('-imageglob',   '*',                                              'shell pattern to match image files in dir')
cmd:option('-enblend',      true,                                            'use enblend to make final image')
cmd:option('-hfov_i',       1.3055,                                          'hfov of camera')
cmd:option('-vfov_i',       1.5290,                                          'vfov of camera')
cmd:option('-scale',        1/16,                                            'scaling factor')
cmd:option('-interactive',  false,                                           'keep cloudlab on')

cmd:text()

params     = cmd:parse(process.argv)

srcdir = params.srcdir
swnum  = params.swdir
imdir  = path.join(srcdir, params.imdir, swnum)
imext  = params.imext
outdir = path.join(srcdir, params.outdir)
util.fs.mkdir_p(outdir)
outdir = path.join(outdir, swnum)
util.fs.mkdir_p(outdir)
outname = path.join(outdir, params.outname)
bldname = path.join(outdir, 'blend_'..params.outname)
smmname = path.join(outdir, 'seam_'..params.outname)

imageglob  = params.imageglob
imagedir   = params.imagedir
enblend    = params.enblend
outimage   = params.outimage

hfov_i    = params.hfov_i
vfov_i    = params.vfov_i
scale     = params.scale

imagefiles = util.fs.files_only(imdir, imext)

function save_image_and_print(imsw, prefix, scale)
  imsw:set_global_parameters(nil,nil,scale,nil,nil)
  local outi,outs,outl,outm  = imsw:get_panorama()
  local score = imsw:get_sift_score()
  pr_phi = math.ceil(imsw.phi_global * 1000000)
  pr_psi = math.ceil(imsw.psi_global * 1000000)
  pr_scr = math.ceil(score * 100000000)
  image.save(outname..'_'..prefix..'_'..pr_scr..'_'..pr_phi..'_'..pr_psi..'_image.png',outi:double())
  print()
  print(prefix, score, iscore, lscore, sscore)
  print()
  print(imsw.phi_global, imsw.psi_global, imsw.hfov_global, imsw.vfov_global)
  print()
  print(imsw.lam_local:repeatTensor(1,1))
  print(imsw.phi_local:repeatTensor(1,1))
  print(imsw.psi_local:repeatTensor(1,1))
  print()
  print()
  outi = nil
  outs = nil
  outl = nil
  outm = nil
  collectgarbage()
end

function blend_panorama(imsw,prefix)
  
	local tmp_fnames = ''
	
	local score = imsw:get_sift_score()
	
	pr_phi = math.ceil(imsw.phi_global * 1000000)
  pr_psi = math.ceil(imsw.psi_global * 1000000)
  pr_scr = math.ceil(score * 100000000)

	for i = 1,imsw.num_images do
		local pfix = ''..i
		while pfix:len() < 3 do
			pfix = '0'..pfix
		end
	
		local outi,outs,outl,outm = imsw:get_frame_output(i)
		outi = image.wand.new(outi):toTensor("double","RGBA","DHW")
		outi[4] = outm[1]
	
		local outfname = outname..'_'..pfix..'.png'
	
		image.save(outfname,outi)
		tmp_fnames = tmp_fnames..' '..outfname
		outi = nil
		outs = nil
		outl = nil
		outm = nil
		collectgarbage()
	
	end

  collectgarbage()
  
	os.execute(string.format("enblend %s -o %s/enblend_%s", tmp_fnames, outdir, params.outname..'_'..prefix..'_'..pr_scr..'_'..pr_phi..'_'..pr_psi..'.png'))
	os.execute(string.format("rm %s", tmp_fnames))
	
	collectgarbage()
	
	--[[
	local pan = imsw:get_blended_panorama()
	image.save(bldname..'_'..prefix..'_'..pr_scr..'_'..pr_phi..'_'..pr_psi..'.png',pan)
	
	local seam = imsw:get_seam_panorama()
	image.save(smmname..'_'..prefix..'_'..pr_scr..'_'..pr_phi..'_'..pr_psi..'.png',seam)
	
	pan = nil
	seam = nil
	--[[]]

	collectgarbage()
end

function save_parameters(imsw)
  local score = imsw:get_sift_score()
  
  torch.save(outname..'_params.dat', { torch.Tensor({score}),
                                       imsw.hfov_global, imsw.vfov_global, imsw.phi_global, imsw.psi_global,
                                       imsw.lam_local, imsw.phi_local, imsw.psi_local })
  collectgarbage()
end


phi = 0
psi = 0
hfov = hfov_i
vfov = vfov_i

_G.imsw = stitcher.ImageSweep.new(imagefiles,hfov_i,vfov_i,params.scale,phi,psi)
save_image_and_print(imsw,'000a',params.scale)

phi = -0.05
psi = -0.0125

imsw:set_global_parameters(hfov,vfov,params.scale,phi,psi)

--[[
local p = torch.load(outname..'_params.dat')
imsw.hfov_global = p[2]
imsw.vfov_global = p[3]
imsw.phi_global  = p[4]
imsw.psi_global  = p[5]
imsw.lam_local   = p[6]
imsw.phi_local   = p[7]
imsw.psi_local   = p[8]
imsw:set_parameters_to_curr()
blend_panorama(imsw,'003'..'e')
collectgarbage()
--[[]]

save_image_and_print(imsw,'000b',params.scale)
--blend_panorama(imsw,'000'..'e')
collectgarbage()

hfov_global_win = 0.0001
vfov_global_win = 0.0001
psi_global_win  = 0.0100
phi_global_win  = 0.0100
psi_local_win   = 0.0100
phi_local_win   = 0.0100
lam_local_win   = 0.0100
iter            = 2--imsw.num_images/2
rep             = 2--imsw.num_images/2
ran_size        = 2--imsw.num_images/2
lp              = 5

--[[]]

for i = 1,lp do

  imsw:optimize_global_all(psi_global_win, phi_global_win, iter, rep, ran_size)

  save_parameters(imsw)
  save_image_and_print(imsw,'00'..i..'a',params.scale)
  collectgarbage()

  imsw:optimize_local_dir_all(psi_local_win, phi_local_win, lam_local_win, iter, rep, ran_size)

  save_parameters(imsw)
  save_image_and_print(imsw,'00'..i..'b',params.scale)
  collectgarbage()

  imsw:optimize_local_all(psi_local_win/2, phi_local_win/2, lam_local_win/2, iter, rep, ran_size)
  
  save_parameters(imsw)
  save_image_and_print(imsw,'00'..i..'c',params.scale)
  collectgarbage()
  
  imsw:optimize_fov_all(hfov_global_win, vfov_global_win, iter, rep, ran_size)
  
  save_parameters(imsw)
  save_image_and_print(imsw,'00'..i..'d',params.scale)
  collectgarbage()
  
  --blend_panorama(imsw,'00'..i..'e')
  
  hfov_global_win = hfov_global_win/2.5
  vfov_global_win = vfov_global_win/2.5
  psi_global_win  = psi_global_win /2.5
  phi_global_win  = phi_global_win /2.5
  psi_local_win   = psi_local_win  /2.5
  phi_local_win   = phi_local_win  /2.5
  lam_local_win   = lam_local_win  /2.5
  
end

collectgarbage()

--[[]]

if not params.interactive then
  collectgarbage()
  process.exit()
end