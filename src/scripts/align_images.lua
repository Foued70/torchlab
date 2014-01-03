path     = require 'path'
blend    = projection.util.blend
optimize = util.optimize.convex_binary_search

Class()

pi  = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Align a linear 360 sweep of images')
cmd:text()
cmd:text('Options')
cmd:option('-imagedir', '/Users/lihui815/Documents/elegant-prize-3149/work/a_00/Images/005', 'directory in which to find images')
cmd:option('-imageglob', '*', 'shell pattern to match image files in dir')
cmd:option('-enblend', true, 'use enblend to make final image')
cmd:option('-outimage', '/Users/lihui815/Documents/elegant-prize-3149/work/a_00/Aligned/005/panorama_360', 'filename of image output')

cmd:text()

-- arg = ''

-- parse input params
params     = cmd:parse(process.argv)
imageglob  = params.imageglob
imagedir   = params.imagedir
enblend    = params.enblend
outimage   = params.outimage

_G.aligner = model.SweepImageAligner.new(imagedir.."/"..imageglob)

-- scales = {0.125,0.25,0.5}
scales = {0.125}
wiggle_mult = 2^3 -- 2^<number of iterations>
current_phi = 0
update = true
for _,s in pairs(scales) do
   collectgarbage()
   aligner:set_scale(s)
   aligner.phi_guess = current_phi
   aligner.delta[2] = current_phi

   -- update current best scores
   score, scores = aligner:compute_scores(update)

   adjusted = ""
   outfname = string.format("%s%s_%.3f_pre.png",outimage:gsub(".png",""),adjusted,s)
   aligner:display_and_save(enblend,outfname)
   
   rad_ppx = aligner:get_input_radians_per_pixel()
   printf(" - rad %f", rad_ppx)
   aligner.lambda_wiggle_base = rad_ppx * wiggle_mult

   printf(" - wiggle %f",aligner.lambda_wiggle_base)
   lmbd,scr,scrs = aligner:find_best_lambda()

   adjusted = adjusted .. "_lambda"
   outfname = string.format("%s%s_%.3f_post.png",outimage:gsub(".png",""),adjusted,s)
   aligner:display_and_save(enblend,outfname)
   
   print("lambda_guess:",      aligner.lambda_guess)
   print("phi guess: ",        aligner.phi_guess)
   print("lambda_wiggle_base", aligner.lambda_wiggle_base)
   print("phi_wiggle_base",    aligner.phi_wiggle_base)
   print("vfov_wiggle_base",   aligner.vfov_wiggle_base)
   print("hfov_wiggle_base",   aligner.hfov_wiggle_base)

   wiggle_mult = wiggle_mult / 2
end

-- quit out of luvit if
if not params.interactive then
   process.exit()
end
