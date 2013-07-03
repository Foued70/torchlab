hough = require 'hough'
saliency = require 'saliency'
require 'image'

dofile '/Users/lihui815/cloudlab/src/sandbox/alignment_utils.lua'


cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compare hough transforms')
cmd:text()
cmd:text('Options')
cmd:option('-image_dir', '/Users/lihui815/cloudlab/src/data/test/96_spring_kitchen/matterport_mount/output', 'image file path to supplementary picture')
cmd:option('-sweepnum', 'sweep_3', 'sweep number')

cmd:text()

if arg == nil then
  arg = ''
end

-- parse input params
params = cmd:parse(arg)

dname = params.image_dir

image_mount = util.fs.glob(dname, "big_img_"..params.sweepnum..".jpg")[1];
swnum = tonumber(string.split(params.sweepnum, 'sweep_')[1])
txnum = swnum-1
image_mptex = util.fs.glob(dname, "0"..txnum..".jpg")[1];

fname_1 = paths.basename(image_mount)
fname_2 = paths.basename(image_mptex)

numrads=2500
numangs=1500
numbest=100
nb = numbest

get_hough(dname, fname_1, image_mount, swnum, numrads, numangs, numbest, nb, "mount")
collectgarbage()
get_hough(dname, fname_2, image_mptex, swnum, numrads, numangs, numbest, nb, "mptex")
collectgarbage()
off = compare_bands(dname, fname_1, fname_2, swnum, nb)

collectgarbage()

