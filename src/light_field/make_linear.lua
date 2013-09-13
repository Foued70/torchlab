Wand  = image.Wand
fs    = require 'fs'
-- Make a lightfield

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Make horizontal and vertical slice light field')
cmd:text()
cmd:text('Options')
cmd:option('-frames_dir',
           "./",
           'base directory for sequence of frames')
cmd:option('-out_dir',
           "output",
           'base directory for output sequence of frames')
cmd:text()


-- parse input params
params = cmd:parse(process.argv)

d = params.frames_dir
outdir = params.out_dir

epi = light_field.Epi.new(d)

print("Saving to "..outdir)

epi:cache(outdir)
