loader = pointcloud.loader
path   = require 'path'

cmd = torch.CmdLine()

cmd:text()
cmd:text('convert to pointcloud dat file')
cmd:text()
cmd:text('Options')
cmd:option('-pcddir', '/Users/lihui815/Documents/precise-transit-6548/work/a/pointcloud', '')
cmd:option('-trfdir', '/Users/lihui815/Documents/precise-transit-6548/work/a/transformations', '')
cmd:option('-swpnum', '001.dat')
cmd:option('-interactive', false, '')
cmd:text()

params     = cmd:parse(process.argv)

pcdfnm = path.join(params.pcddir, params.swpnum)
trffnm = path.join(params.trfdir, 'sweep'..params.swpnum..'a')

print(pcdfnm)
print(trffnm)

pc = torch.load(pcdfnm)
tr = torch.load(trffnm)

pc:set_transformation_matrix(tr)
pc:save_to_data_file(pcdfnm)

if not params.interactive then
  collectgarbage()
  process.exit()
end