path = require 'path'
blend = projection.util.blend
optimize = util.optimize.convex_binary_search

Class()

pi  = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagedir', './', 'directory in which to find images')
cmd:option('-imageglob', '*png', 'shell pattern to match image files in dir')
cmd:option('-enblend', false, 'use enblend to make final image')
cmd:option('-outimage', 'panorama_360.png', 'filename of image output')

cmd:text()

-- arg = ''

-- parse input params
params     = cmd:parse(process.argv)
imageglob  = params.imageglob
imagedir   = params.imagedir
enblend    = params.enblend
outimage   = params.outimage

_G.aligner = model.SweepImageAligner.new(imagedir.."/"..imageglob)


wiggle_mult = 2^3 -- 2^<number of iterations>
current_phi = 0
update = true
for _,s in pairs({0.125,0.25,0.5}) do
   collectgarbage()
   aligner:set_scale(s)

   -- update current best scores
   score, scores = aligner:compute_scores(update)

   adjusted = ""
   outfname = string.format("%s%s_%.2f.png",outimage:gsub(".png",""),adjusted,s)
   aligner:display_and_save(enblend,outfname)
   
   rad_ppx = aligner:get_input_radians_per_pixel()
   printf(" - rad %f", rad_ppx)
   aligner.lambda_wiggle_base = rad_ppx * wiggle_mult

   printf(" - wiggle %f",aligner.lambda_wiggle_base)
   lmbd,scr,scrs = aligner:find_best_lambda()

   adjusted = adjusted .. "_lambda"
   outfname = string.format("%s%s_%.2f.png",outimage:gsub(".png",""),adjusted,s)
   aligner:display_and_save(enblend,outfname)

   wiggle_mult = wiggle_mult / 2
end

-- quit out of luvit if
if not params.interactive then
   process.exit()
end
