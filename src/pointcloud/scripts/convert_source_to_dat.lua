loader = pointcloud.loader
path   = require 'path'

cmd = torch.CmdLine()

cmd:text()
cmd:text('convert to pointcloud dat file')
cmd:text()
cmd:text('Options')
cmd:option('-srcdir', '/Users/lihui815/Documents/elegant-prize-3149/source/po_scan/a/005', '')
cmd:option('-prjdir', '/Users/lihui815/Documents/elegant-prize-3149', '')
cmd:option('-wrkdir', 'work',   '')
cmd:option('-pcddir', 'pointcloud', '')
cmd:option('-interactive', false, '')
cmd:text()

params     = cmd:parse(process.argv)

srcdir = path.join(params.srcdir)
swpnum = path.basename(srcdir)
tmp    = path.dirname(srcdir)
vernum = path.basename(tmp)
prjdir = path.dirname(path.dirname(path.dirname(tmp)))
wrkdir = path.join(prjdir, params.wrkdir)
verdir = path.join(wrkdir, vernum)
pcddir = path.join(verdir, params.pcddir)
pcdfnm = path.join(pcddir, swpnum..'.dat')

if not util.fs.is_dir(wrkdir) then
  util.fs.mkdir_p(wrkdir)
end
if not util.fs.is_dir(verdir) then
  util.fs.mkdir_p(verdir)
end
if not util.fs.is_dir(pcddir) then
  util.fs.mkdir_p(pcddir)
end

pc = loader.load_pobot_ascii(srcdir)
pc:save_to_data_file(pcdfnm)

if not params.interactive then
  collectgarbage()
  process.exit()
end
