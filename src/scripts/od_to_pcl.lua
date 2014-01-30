io = require 'io'
path = require 'path'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('convert to ply')
cmd:text()
cmd:text('Options')
cmd:option('-prjdir','/Users/lihui815/Documents/elegant-prize-3149','')
cmd:option('-interactive',false)
-- test new itrw options
cmd:text()

params = cmd:parse(process.argv)

prjdir = params.prjdir

srcdir = path.join(prjdir,'work/a_00/OD' ):gsub('/*$','')
outdir = path.join(prjdir,'work/a_00/PLY'):gsub('/*$','')

pcfiles               = util.fs.glob(srcdir, {"od$"})

util.fs.mkdir_p(outdir)

for j=1,#pcfiles do
  p = pcfiles[j]
  o = path.join(outdir,path.basename(p,'.od')..'.ply')
  pc = PointCloud.Pointcloud.new(p)
  pts = pc.points
  rgb = pc.rgb
  nmp = pc:get_normal_map()
  nml = torch.zeros(3,pc.count)
  nml[1] = nmp[1][pc.mask_map:eq(0)]:clone()
  nml[2] = nmp[2][pc.mask_map:eq(0)]:clone()
  nml[3] = nmp[3][pc.mask_map:eq(0)]:clone()
  nml = nml:t():clone()
  mask = pc.rgb:gt(0):double():sum(2):squeeze()
  cnt = mask:gt(0):double():sum()
  out = io.open(o, 'w')
  out:write('ply\n')
  out:write('format ascii 1.0\n')
  out:write('element vertex '..cnt..'\n')
  out:write('property float x\n')
  out:write('property float y\n')
  out:write('property float z\n')
  out:write('property float nx\n')
  out:write('property float ny\n')
  out:write('property float nz\n')
  out:write('property uchar red\n')
  out:write('property uchar green\n')
  out:write('property uchar blue\n')
  out:write('end_header\n')
  for i=1,pc.count do
    if mask[i] > 0 then
      pt = pts[i]
      nm = nml[i]
      cl = rgb[i]
      out:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..nm[1]..' '..nm[2]..' '..nm[3]..' '..cl[1]..' '..cl[2]..' '..cl[3]..'\n')
    end
  end
  out:close()
end

if (not params.interactive) then
   process.exit()
end