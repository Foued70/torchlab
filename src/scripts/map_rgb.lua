path = require 'path'
Pointcloud = PointCloud.Pointcloud

for sw = 10,71 do

if sw ~= 18 and sw ~= 20 and sw ~= 36 then

topdir = '/Users/lihui815/Documents'
prjdir = 'mobile-void-0590'--'elegant-prize-3149'--
srcdir = 'source/po_scan/a/'
wrkdir = 'work/a_00'
imgdir = 'Aligned'
sweepn = '0'..sw

util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'NMP'))
util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'RGB'))
util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'XYZRGB'))
util.fs.mkdir_p(path.join(topdir,prjdir,wrkdir,'OD'))

pcfile = path.join(topdir,prjdir,srcdir,sweepn,'sweep.xyz')
imfile = path.join(topdir,prjdir,wrkdir,imgdir,sweepn,'enblend_panorama_360_lambda_0.125_post.png')
nmfile = path.join(topdir,prjdir,wrkdir,'NMP/nmp'..sweepn..'.png')
clfile = path.join(topdir,prjdir,wrkdir,'RGB/rgb'..sweepn..'.png')
xyfile = path.join(topdir,prjdir,wrkdir,'XYZRGB/xyz'..sweepn..'.xyz')
odfile = path.join(topdir,prjdir,wrkdir,'OD/pc'..sweepn..'.od')

pc = PointCloud.PointCloud.new(pcfile)
img = image.load(imfile)
ih = img:size(2)
iw = img:size(3)
rad_per_pix = 2*math.pi/iw
hhov = 2*math.pi
vhov = ih*rad_per_pix
vcnt = vhov/2
thof = 45*math.pi/64--37*math.pi/64
hhof = 0

xyz_map = pc:get_xyz_map_no_mask()
index,rmask = pc:get_index_and_mask()
phi,theta = pc:get_xyz_phi_theta()
nmp,_G.ndd = pc:get_normal_map()
emask = rmask:eq(0)

xyz = xyz_map:clone()
msk = theta:le(0):cmul(emask)
theta[msk] = theta[msk] + 2*math.pi
theta = theta*(-1)+2*math.pi
theta[emask] = theta[emask] + thof
msk = theta:gt(2*math.pi):cmul(emask)
theta[msk] = theta[msk] - 2*math.pi
theta[rmask] = 0

xyz[3] = xyz[3]+hhof
xyz[3][rmask] = 0
depth = xyz:norm(2,1):squeeze()
phi = xyz[3]:clone():cdiv(depth):asin()
phi[rmask]= 0
phi = phi*(-1)+vcnt

rrr = torch.zeros(pc.height,pc.width)
ggg = torch.zeros(pc.height,pc.width)
bbb = torch.zeros(pc.height,pc.width)
msk = torch.zeros(pc.height,pc.width)

im_r = img[1]
im_g = img[2]
im_b = img[3]

--[[]]

for h = 1,pc.height do
  ph = phi[h]
  th = theta[h]
  mh = rmask[h]
  for w = 1,pc.width do
    if mh[w] == 0 then
      p = ph[w]
      t = th[w]
      if p >= 0 and p < vhov then
        imh = math.floor(p/rad_per_pix+1)
        imw = math.floor(t/rad_per_pix+1)
        if imh < 1 or imw < 1 or imh > ih or imw > iw then
          print(h,w,p,t,imh,imw,ih,iw)
        else
          rrr[h][w] = im_r[imh][imw]
          ggg[h][w] = im_g[imh][imw]
          bbb[h][w] = im_b[imh][imw]
          msk[h][w] = 1
        end
      end
    end
  end
end

rgb_map = torch.zeros(3,pc.height,pc.width)
rgb_map[1] = rrr
rgb_map[2] = ggg
rgb_map[3] = bbb
msk[rmask] = 0
num = msk:double():sum()

nmpi = image.combine(nmp)
image.save(nmfile,nmpi)
image.save(clfile,rgb_map)

pc:load_rgb_map(clfile)

points_t = torch.zeros(3,num)
rgb_t = torch.zeros(3,num)

for i = 1,3 do
  points_t[i] = xyz_map[i][msk:byte()]
  rgb_t[i] = rgb_map[i][msk:byte()]
end
rgb_t:mul(255):floor()

points = points_t:t()
rgb = rgb_t:t()
pc.save_any_points_to_xyz(xyfile,points,rgb)
pc:save_to_od(odfile)

--[[]]
end

collectgarbage()
end
