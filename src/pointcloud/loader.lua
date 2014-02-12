local io  = require 'io'
local path = require 'path'
local pcd = pointcloud.pointcloud
local ffi = require 'ffi'
local ctorch = util.ctorch

ffi.cdef
[[
    int get_valid_mask(char* valid_mask, short* hwindices, long length, int height, int width)
]]

local libpc   = util.ffi.load('libpcd')


loader = Class()

local faro_degree_above = 90
local faro_degree_below = 60
local default_minradius = 0.01
local default_maxradius = 25.0


function loader.fix_newline(fname)
    local dname = path.dirname(fname)
    local ename = path.extname(fname)
    local bname = path.basename(fname,ename)
	local fname_tmp = path.join(dname,bname..'_tmp'..ename)
	
	local file_in = io.open(fname)
	local file_tp = io.open(fname_tmp,'w')
	local ct = 0
	local blc = 0
	local flag = true
	
	for block in file_in:lines() do
	  
	  if blc == 0 then
	    local numtok = 0
  	    for token in string.gmatch(block, "[^\r]+") do
  	      numtok = numtok + 1
  	      if numtok > 1 then
  	        flag = false
  	        break
  	      end
  	    end
  	    collectgarbage()
  	  end
  	  
  	  if flag then
  	    break
  	  end
  	  
	  for token in string.gmatch(block, "[^\r]+") do
	    if ct > 0 then
  	      file_tp:write('\n'..token)
  	    else
  	      file_tp:write(token)
  	    end
  	    ct = ct + 1
	  end
	  
	  collectgarbage()
	  blc = blc + 1
	  
	end
	
	if not flag then
	  util.fs.exec('cp '..fname_tmp..' '..fname)
	end
	  util.fs.exec('rm '..fname_tmp)
	
	collectgarbage()
	file_in:close()
	file_tp:close()
	collectgarbage()
end

function loader.load_pobot_meta_data(filename)
  local metafile = io.open(filename,'r')
  local aline = metafile:read()
  local properties = {}
  while (aline) do
    local asplit = string.split(aline,':')
    local propname = asplit[1]
    local propvalu = asplit[2]
    local propname = propname:sub(1,#propname-1)
    properties[propname]=tonumber(propvalu)
    aline = metafile:read()
  end
  metafile:close()
  return properties
end

function loader.fix_meta(depth_map,meta)
  local predicted_good = math.floor((2*math.pi-meta.azimuth_per_point*depth_map:size(1))/(meta.azimuth_per_line))
  local minValue = math.huge
  local bestI = predicted_good
  for i=math.max(1, predicted_good-10), math.min(depth_map:size(2), predicted_good+10) do
    local error = (depth_map:sub(1,-1,1,1)-depth_map:sub(1,-1,i,i)):pow(2):sum()
    if(error<minValue) then
      minValue = error
      bestI = i
    end
  end
  meta.azimuth_per_line = 2*math.pi/(bestI-1)
  return meta
end

function loader.load_pobot_ascii(dirname, minradius, maxradius)

  minradius = minradius or default_minradius
  maxradius = maxradius or default_maxradius
  
  local metaname = path.join(dirname,'sweep.meta')
  local rawname  = path.join(dirname,'sweep.raw')
  
  -- fix return carriages
  loader.fix_newline(metaname)
  loader.fix_newline(rawname)
  
  -- load raw file
  local file = io.open(rawname, 'r');
  local line = file:read();
  file:close()
  
  -- check validity
  local is_valid = true
  local err_str  = ''
  if line == nil then
    is_valid = false
    err_str = 'load_pobot_ascii: nil file: '..rawname
  else
    local countColumns = 0
    for token in string.gmatch(line, '[^%s]+') do
      countColumns = countColumns + 1
    end
    if countColumns ~= 1 then
      is_valid = false
      err_str = 'load_pobot_ascii: expected 1 columns, got '..countColumns
    end
  end
  if (not is_valid) then
    collectgarbage()
    error(err_str)
  end
  
  -- read the meta-file
  local meta = loader.load_pobot_meta_data(metaname)
	local height = meta.h
	local width  = meta.w
  
  -- get the count
  local count = 0
	local totalLines = util.fs.exec('wc -l '..rawname)
	for token in string.gmatch(totalLines, "[^%s]+") do
		count = tonumber(token)
		break
	end
	
	-- read file
	local file     = torch.DiskFile(rawname, 'r', false) 
  local dpt_list = torch.Tensor(torch.File.readDouble(file,count)):reshape(count)
  file:close()
  
  -- set meter and center
  local meter = 1000
  minradius   = minradius * meter
  maxradius   = maxradius * meter
  centroid    = torch.zeros(3)
  
  --maps
  local depth_map        = util.torch.flipTB(dpt_list:reshape(width,height):t()):clone():contiguous()
  
  --this will later be fixed in the data we get, but for now, recalc best min error width to get new azimuth per line
  meta = loader.fix_meta(depth_map, meta)

  local rgb_map          = torch.zeros(height,width):clone():contiguous()
  local intensity_map    = torch.zeros(height,width):clone():contiguous()

  local xyz_phi_map      = torch.range(0,height-1):repeatTensor(width,1)
  xyz_phi_map            = util.torch.flipTB(xyz_phi_map:t())
  xyz_phi_map            = xyz_phi_map:mul(meta.elevation_per_point):add(meta.elevation_start+math.pi/2)
  xyz_phi_map            = xyz_phi_map:clone():contiguous()
  
  local xyz_theta_map_pe = torch.range(0,height-1):repeatTensor(width,1)
  xyz_theta_map_pe       = xyz_theta_map_pe:t()
  xyz_theta_map_pe       = xyz_theta_map_pe:mul(-meta.azimuth_per_point)
  
  local xyz_theta_map_pl = torch.range(0,width-1):repeatTensor(height,1)
  xyz_theta_map_pl       = xyz_theta_map_pl:mul(-meta.azimuth_per_line)
  
  local xyz_theta_map    = xyz_theta_map_pe + xyz_theta_map_pl + math.pi
  local msk              = torch.zeros(xyz_theta_map:size())
  while xyz_theta_map:max() >  math.pi do
    msk              = xyz_theta_map:gt(math.pi)
    xyz_theta_map[msk]     = xyz_theta_map[msk]-(math.pi*2)
  end
  while xyz_theta_map:min() <= -math.pi do
    msk                    = xyz_theta_map:le(-math.pi)
    xyz_theta_map[msk]     = xyz_theta_map[msk]+(math.pi*2)
  end
  xyz_theta_map          = xyz_theta_map:clone():contiguous()
  
  msk                    = nil
  xyz_theta_map_pe       = nil
  xyz_theta_map_pl       = nil
  collectgarbage()
  
  -- only use stuff that's between min and max radius
  local depth_valid_mask = depth_map:ge(minradius):cmul(depth_map:le(maxradius))
  local imask            = depth_valid_mask:eq(0)
  count                  = depth_valid_mask:double():sum()
  depth_map[imask]       = 0
  rgb_map[imask]         = 0
  intensity_map[imask]   = 0
  xyz_phi_map[imask]     = 0
  xyz_theta_map[imask]   = 0
  
  -- masks
  local rgb_valid_mask = torch.zeros(height,width):byte()
  local intensity_valid_mask = torch.zeros(height,width):byte()
  
  imask = nil
  collectgarbage()
  
  -- load inputs
  input = {}
  
  input.meta                 = meta
  input.count                = count
  input.height               = height
  input.width                = width
  input.meter                = meter
  input.centroid             = centroid
  input.depth_map            = depth_map
  input.rgb_map              = rgb_map
  input.intensity_map        = intensity_map
  input.xyz_phi_map          = xyz_phi_map
  input.xyz_theta_map        = xyz_theta_map
  input.depth_valid_mask     = depth_valid_mask
  input.rgb_valid_mask       = rgb_valid_mask
  input.intensity_valid_mask = intensity_valid_mask
  
  local pc = pcd.new(input)
  
  input                = nil
  meta                 = nil
  count                = nil
  height               = nil
  width                = nil
  meter                = nil
  centroid             = nil
  depth_map            = nil
  rgb_map              = nil
  intensity_map        = nil
  xyz_phi_map          = nil
  xyz_theta_map        = nil
  depth_valid_mask     = nil
  rgb_valid_mask       = nil
  intensity_valid_mask = nil
  collectgarbage()
  
  -- return
  return pc
  --[[]]
  
end

function loader.load_pobot_binary(dirname, minradius, maxradius)
  
  minradius = minradius or default_minradius
  maxradius = maxradius or default_maxradius
  
  local metaname = path.join(dirname,'sweep.meta')
  local rawname  = path.join(dirname,'sweep.raw')
  
  -- fix return carriages
  loader.fix_newline(metaname)
  
  -- load meta data
  local meta = loader.load_pobot_meta_data(metaname)
  
  
  
  -- set input list
  local input = {}
  input.meta = meta
  
  -- create new pointcloud object
  local pc = pcd.new(input)
  
  -- collect garbage
  meta = nil
  collectgarbage()
  
  return pc
  
end

function loader.load_faro_xyz(filename, minradius, maxradius)
  
  minradius = minradius or default_minradius
  maxradius = maxradius or default_maxradius
  
  loader.fix_newline(filename)
  
  -- load raw file
  local file = io.open(filename, 'r');
  local line = file:read();
  file:close()
  
  -- check validity
  local is_valid = true
  local err_str  = ''
  if line == nil then
    is_valid = false
    err_str = 'load_faro_xyz: nil file: '..filename
  else
    local countColumns = 0
    for token in string.gmatch(line, '[^%s]+') do
      countColumns = countColumns + 1
    end
    if countColumns ~= 8 then
      is_valid = false
      err_str = 'load_faro_xyz: expected 8 columns, got '..countColumns
    end
  end
  if (not is_valid) then
    collectgarbage()
    error(err_str)
  end
  
  -- get the count
	local count = 0
	local totalLines = util.fs.exec('wc -l '..filename)
	for token in string.gmatch(totalLines, "[^%s]+") do
		count = tonumber(token)
		break
	end
	
	-- read file
	local file = torch.DiskFile(filename, 'r', false) 
  local hwxyzrgb =torch.Tensor(torch.File.readDouble(file,8*count)):reshape(count, 8)
  file:close()
  
  -- get info from file
  local meter    = 1000
  minradius      = minradius * meter
  maxradius      = maxradius * meter
  local centroid = torch.zeros(3)
  local h, w
  local x, y, z
  local r, g, b
  h = hwxyzrgb:select(2,1):clone():add(1)
  w = hwxyzrgb:select(2,2):clone():add(1)
  x = hwxyzrgb:select(2,3):clone():mul(meter):floor()
  y = hwxyzrgb:select(2,4):clone():mul(meter):floor()
  z = hwxyzrgb:select(2,5):clone():mul(meter):floor()
  r = hwxyzrgb:select(2,6):clone():div(255)
  g = hwxyzrgb:select(2,7):clone():div(255)
  b = hwxyzrgb:select(2,8):clone():div(255)
  local height = h:max()
  local width  = w:max()
  hwxyzrgb = nil
  collectgarbage()
  
  -- adjust to center
  local rowid       = math.floor((faro_degree_above / (faro_degree_above + faro_degree_below)) * height)
  local indexCenter = h:eq(rowid)
  local midrow      = z[indexCenter]
  local meanz       = midrow[midrow:gt(0)]:mean()
  z                 = z-meanz
  indexCenter       = nil
  midrow            = nil
  collectgarbage()
  
  -- find the depth tensor
  local dpt_list    = x:clone():pow(2):add(y:clone():pow(2)):add(z:clone():pow(2)):sqrt()
  
  -- only use stuff that's between min and max radius
  local select_mask = dpt_list:le(maxradius):cmul(dpt_list:ge(minradius))
  count             = select_mask:double():sum()
  h                 = h[select_mask]:clone()
  w                 = w[select_mask]:clone()
  x                 = x[select_mask]:clone()
  y                 = y[select_mask]:clone()
  z                 = z[select_mask]:clone()
  r                 = r[select_mask]:clone()
  g                 = g[select_mask]:clone()
  b                 = b[select_mask]:clone()
  dpt_list          = dpt_list[select_mask]:clone()
  
  -- make lists
  local xyz_phi_list   = z:clone():cdiv(dpt_list):asin()
  local xyz_theta_list = x:clone():cdiv(x:clone():pow(2):add(y:clone():pow(2)):sqrt()):acos()
  local xyz_theta_neg  = y:clone():cdiv(x:clone():pow(2):add(y:clone():pow(2)):sqrt()):asin():lt(0)
  xyz_theta_list[xyz_theta_neg] = -xyz_theta_list[xyz_theta_neg]
  
  local hwindices      = torch.cat(h,w,2):short()
  
  -- find depth_valid_mask
  local depth_valid_mask = torch.zeros(height,width):byte():clone():contiguous()
  libpc.get_valid_mask(torch.data(depth_valid_mask), torch.data(hwindices:clone():contiguous()), 
                            count, height, width)
  
  -- make maps
  local depth_map     = torch.zeros(height, width)
  local xyz_phi_map   = torch.zeros(height, width)
  local xyz_theta_map = torch.zeros(height, width)
  local rgb_map       = torch.zeros(3, height, width)
  local xyz_map       = torch.zeros(3, height, width)
  
  local vmsk = depth_valid_mask:clone()
  
  depth_map[vmsk]     = dpt_list:clone()
  xyz_phi_map[vmsk]   = xyz_phi_list:clone()
  xyz_theta_map[vmsk] = xyz_theta_list:clone()
  rgb_map[1][vmsk]    = r:clone()
  rgb_map[2][vmsk]    = g:clone()
  rgb_map[3][vmsk]    = b:clone()
  xyz_map[1][vmsk]    = x:clone()
  xyz_map[2][vmsk]    = y:clone()
  xyz_map[3][vmsk]    = z:clone()
  
  local wnd = image.Wand.new(rgb_map:clone())
  wnd:colorspace('LAB')
  local intensity_map = wnd:toTensor('double',nil,'DHW')[1]:clone()
  
  local rgb_valid_mask       = depth_valid_mask:clone()
  local intensity_valid_mask = depth_valid_mask:clone()
  
  dpth_list      = nil
  xyz_phi_list   = nil
  xyz_theta_list = nil
  xyz_theta_neg  = nil
  hwindices      = nil
  h = nil
  w = nil
  x = nil
  y = nil
  z = nil
  r = nil
  g = nil
  b = nil
  vmsk = nil
  collectgarbage()
  
  -- load inputs
  input = {}
  
  input.count                = count
  input.height               = height
  input.width                = width
  input.meter                = meter
  input.centroid             = centroid
  input.depth_map            = depth_map
  input.rgb_map              = rgb_map
  input.intensity_map        = intensity_map
  input.xyz_phi_map          = xyz_phi_map
  input.xyz_theta_map        = xyz_theta_map
  input.depth_valid_mask     = depth_valid_mask
  input.rgb_valid_mask       = rgb_valid_mask
  input.intensity_valid_mask = intensity_valid_mask
  input.xyz_map              = xyz_map
  
  local pc = pcd.new(input)
  
  input                = nil
  count                = nil
  height               = nil
  width                = nil
  meter                = nil
  centroid             = nil
  depth_map            = nil
  rgb_map              = nil
  intensity_map        = nil
  xyz_phi_map          = nil
  xyz_theta_map        = nil
  depth_valid_mask     = nil
  rgb_valid_mask       = nil
  intensity_valid_mask = nil
  xyz_map              = nil
  collectgarbage()
  
  -- return
  return pc
  
end