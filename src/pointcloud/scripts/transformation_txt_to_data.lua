path   = require 'path'
io     = require 'io'

cmd = torch.CmdLine()

cmd:text()
cmd:text('convert to pointcloud dat file')
cmd:text()
cmd:text('Options')
cmd:option('-readdir', '/Users/lihui815/Documents/precise-transit-6548/work/a/transformations_txt', '')
cmd:option('-writdir', '/Users/lihui815/Documents/precise-transit-6548/work/a/transformations', '')
cmd:option('-fname', 'sweep001_transf.txt', '')
cmd:option('-interactive', false, '')
cmd:text()

params     = cmd:parse(process.argv)

util.fs.mkdir_p(params.writdir)

basename = path.basename(params.fname, '_transf.txt')
readfile = path.join(params.readdir, params.fname)
writfile = path.join(params.writdir, basename..'.data')

file = io.open(readfile, 'r')

trans = torch.zeros(16)
for i = 1,16 do
  trans[i] = file:read('*num')
end

trans = trans:reshape(4,4):clone()

torch.save(writfile,trans)

file:close()

if not params.interactive then
  collectgarbage()
  process.exit()
end

