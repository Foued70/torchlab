pi = math.pi
pi2 = pi/2
io               = require 'io'
saliency         = require "../../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual

src_dir = 'arcs/temporary-circle-6132/source/po_scan/a/001/'
wrk_dir = src_dir:gsub("source","work")

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000
tree_resolution = 15

pcfile = src_dir .. 'sweep.xyz'
rawfile = src_dir .. 'sweep.raw'

_G.pc   = PointCloud.PointCloud.new(pcfile, max_radius)
_G.normals = pc:get_normal_map()

base_win    = 13
scalefactor = 2
nscale      = 3

_G.salient,_G.sc = saliency.high_entropy_features{img=normals,
                                                  kr=base_win,kc=base_win,
                                                  scalefactor=scalefactor,
                                                  nscale=nscale}
-- _G.inv_salient = salient:clone():add(-salient:max()):abs() -- we want the inverse saliency
_G.inv_salient = sc[3]:add(-sc[3]:max()):abs()
final_win = base_win * scalefactor ^ (nscale -1)
_G.nms    = image.NonMaximalSuppression.new(final_win,final_win)
_G.mx     = nms:forward(inv_salient):squeeze()

printf("max mx = %f", mx:max())
mx:mul(1/mx:max()) -- max == 1
printf("max mx = %f", mx:max())
_G.bmx    = mx:gt(0)
k         = bmx[{{1,bmx:size(1)-final_win},{1,bmx:size(2)-final_win}}]:sum()
printf("found %d minimum regions", k)

_G.idx       = torch.LongTensor(2,k)
_G.val       = torch.Tensor(k)
-- TODO in C ackkkkk
c = 0
for i = 1,bmx:size(1)-final_win do
   for j = 1,bmx:size(2)-final_win do
      if mx[i][j] > 0 then
         c = c + 1
         idx[1][c] = i
         idx[2][c] = j
         val[c] = mx[i][j]
      end
   end
end
percent_keep = 0.5
printf("found %d keeping %d",c, c*percent_keep)

s,si = torch.sort(val)

imgh = bmx:size(1)
imgw = bmx:size(2)
w    = final_win
hw   = final_win / 2


xyz = pc:get_xyz_map()

allpts = xyz:reshape(xyz:size(1),xyz:size(2)*xyz:size(3)):t():contiguous()
_G.wins  = {}
_G.masks = {}
_G.imgs  = {}
threshold = 5 -- in mm
for ii = c,1,-1 do
   rgb = normals:clone():add(1):mul(0.5)
   i = si[ii]
   local low_h = math.max(1,idx[1][i]-hw)
   local low_w = math.max(1,idx[2][i]-hw)
   local far_h = math.min(imgh-hw,low_h+w)
   local far_w = math.min(imgw-hw,low_w+w)
   local win   = xyz[{{},{low_h,far_h},{low_w,far_w}}]
   _G.pts   = win:reshape(win:size(1),win:size(2)*win:size(3)):t():contiguous()

   n,d = fit_plane(pts)
   _G.res = compute_residual(pts,n,d)
   if res:std() < threshold then
      printf("[%d] saliency: %f residual mean: %f, std: %f",ii,val[i],res:mean(), res:std())
      _G.allres = compute_residual(allpts,n,d)
      planepts_mask = allres:lt(threshold):resize(imgh,imgw)

      rgb[1][planepts_mask] = 1
      rgb[2][planepts_mask] = 1
      rgb[3][planepts_mask] = 1
      rgb[{{},{low_h,far_h},{low_w,low_w+1}}] = 0
      rgb[{{},{low_h,low_h+1},{low_w,far_w}}] = 0
      rgb[{{},{low_h,far_h},{far_w,far_w+1}}] = 0
      rgb[{{},{far_h,far_h+1},{low_w,far_w}}] = 0
      image.display(rgb)

      table.insert(masks,planepts_mask)
      table.insert(imgs,rgb:squeeze())
      table.insert(wins,win)
   end
end
