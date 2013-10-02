local path = require 'path'
local pcl = PointCloud.PointCloud
local scan = align_floors_endtoend.Scan
local leafsize = 0.01

local a = PointCloud.PointCloud.new('/Users/lihui815/cloudlab/build/usr/local/tmp/arcs/virtuous-walk-1066/work/a_02/OD/72.od')

local xyz = a:get_xyz_map_no_mask():clone()
xyz:apply(function(x) 
  if (x==math.huge) then 
    return 0 
  end 
end)

local hh = a.height
local ww = a.width

local nmp = a:get_normal_map()
local aa = nmp[1]:clone()
local bb = nmp[2]:clone()
local cc = nmp[3]:clone()
local norm = aa:clone():pow(2):add(bb:clone():pow(2)):add(cc:clone():pow(2)):sqrt()

local dd = xyz:clone():cmul(nmp):sum(1):squeeze():mul(-1)

local size_ab = 721
local size_dd = 2001

local scal_ab = 2*math.pi/(size_ab-1)
local scal_dd = (dd:max()-dd:min())/(size_dd-1)

print(scal_ab,scal_dd)

local trans = torch.zeros(size_ab,size_dd)

local coord_d = dd:clone():add(-dd:min()):div(scal_dd):floor():add(1)

for i=1,hh do
  local ca = aa[i]
  local cb = bb[i]
  local cd = coord_d[i]
  local c  = cc[i]
  local nm = norm[i]
  for j = 1,ww do
    if math.abs(c[j]) < 0.01 and nm[j] > 0 then
      local aaa = ca[j]
      local bbb = cb[j]
      local the = math.acos(aaa)
      
      if bbb < 0 then
        the = - math.acos(aaa) + 2*math.pi
      end
      
      the = math.floor(the/scal_ab) + 1
      
      local ddd = cd[j]
      trans[the][ddd] = trans[the][ddd]+1
      aaa = nil
      bbb = nil
      ddd = nil
    end
  end
  collectgarbage()
  ca = nil
  cb = nil
  cd = nil
  c = nil
  nm = nil
end
collectgarbage()
trans:div(trans:max())

image.display(trans)