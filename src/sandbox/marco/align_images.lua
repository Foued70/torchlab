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
cmd:option('-scale', 1, 'how much to down scale original image')
cmd:option('-enblend', false, 'use enblend to make final image')
cmd:option('-outimage', 'output_360.png', 'filename of image output')

cmd:text()

-- arg = ''

-- parse input params
params     = cmd:parse(process.argv)
imageglob  = params.imageglob
imagedir   = params.imagedir
scale      = params.scale
enblend    = params.enblend
outimage   = params.outimage

aligner = model.SweepImageAligner.new(imagedir.."/"..imageglob)
-- aligner:set_scale(0.2)

aligner:find_best_lambda()
aligner:display_and_save(enblend,outimage)

