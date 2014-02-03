local ufs = util.fs
local path = require 'path'
ffi  = require 'ffi'
local ctorch = util.ctorch
local log = require '../util/log'

cmd = torch.CmdLine()
cmd:text()

-- parse input params
params = cmd:parse(process.argv)


ffi.cdef [[
void compute_scores(double * score, 
                    THDoubleTensor * x, THDoubleTensor * y, THDoubleTensor * z, 
                    double * eqn, double res_thresh);
]]

local libpc   = util.ffi.load('libpointcloud')


pc = PointCloud.PointCloud.new('/Users/lihui815/Documents/motor-unicorn-0776/work/planes/sweep_001/sweep_001.od')
xyz_map = pc:get_xyz_map_no_mask()
nmp_map, ndd_map = pc:get_normal_map()
index,rmask = pc:get_index_and_mask()
emask = rmask:eq(0)
h = pc.height
w = pc.width
c = pc.count

eqn_map = torch.zeros(4,h,w):clone():contiguous()
eqn_map[4] = -ndd_map

scr_map = torch.zeros(h,w)

print('computing')
log.tic()
libpc.compute_scores(torch.data(scr_map),torch.cdata(xyz_map[1]:clone():contiguous()),
                                         torch.cdata(xyz_map[2]:clone():contiguous()),
                                         torch.cdata(xyz_map[3]:clone():contiguous()),
                                         torch.data(eqn_map),10)
sec = log.toc()/1000
min = math.floor(sec/60)
sec = sec - min*60
print(min,sec)

min_scr = scr_map:min()
max_scr = scr_map:max()

_G.scr_map = -scr_map+max_scr


image.display(scr_map:clone():div(scr_map:max()))

