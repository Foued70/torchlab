Wand  = image.Wand
fs    = require 'fs'
-- Process a lightfield

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
outdir = d.."/"..params.out_dir

save_frames = false
files       = util.fs.glob(outdir,{"t7"})
nfiles      = #files

print(files)
i = 0
for _,fpath in pairs(files) do
   -- fpath = d..f
   if fs.existsSync(fpath) and fs.statSync(fpath).is_file then
      log.tic()
      i = i+1
      print("["..i.."/"..nfiles.."] "..fpath)
      local epi = torch.load(fpath)
      epi = epi:transpose(4,1):contiguous() -- nchannels, nrows, ncols, nimages.
      print(log.toc() * 1e-3 .. " sec")
      print(epi:size())
      if i%10 == 0 then collectgarbage() end
   end
end

