io = require 'io'
path = require 'path'

odfile = "input/1000.xyz"
pngfile = "input/office_test-1000-1001/1000.png"

bname = path.basename(odfile):gsub(".","_")

outply = odfile .. ".ply"
outjs  = odfile .. ".js"
 
_G.pc = PointCloud.PointCloud.new(odfile)
pc:load_rgb_map(pngfile)

_G.xyz = pc:get_xyz_map()
fpose = pc:estimate_faro_pose()
xyz[3]:add(-fpose[3])

_G.nm, _, mask = pc:get_smooth_normal()
nm:mul(-1)
_G.valid = mask:eq(0)
_G.npts = valid:sum()

_G.rgb = pc:get_rgb_map()

_G.d = torch.Tensor(9,npts)

d[1] = xyz[1][valid]*1e-3
d[2] = xyz[2][valid]*1e-3
d[3] = xyz[3][valid]*1e-3
d[4] = nm[1][valid]
d[5] = nm[2][valid]
d[6] = nm[3][valid]
d[7] = rgb[1][valid]
d[7]:mul(1/255)
d[8] = rgb[2][valid]
d[8]:mul(1/255)
d[9] = rgb[3][valid]
d[9]:mul(1/255)



_G.d = d:t():contiguous()

function save_js_for_luma()
   objf = assert(io.open(outjs, "w"))
   objf:write(string.format("var %s = [\n",bname))
   i = 1
   d:apply(function (x) 
              objf:write(string.format("%f,",x)) 
              if (i%9 == 0) then objf:write("\n") end 
              i = i+1 
           end)
   objf:write("];")
   objf:close()
end

-- now dump ply
function save_ply_for_potree()
   objf = assert(io.open(outply, "w"))
   objf:write ( [[ply
format ascii 1.0
comment author: Floored Inc.
comment object: colored pointcloud
element vertex ]] .. npts ..
                [[

property float x
property float y
property float z
property float Nx
property float Ny
property float Nz
property float red
property float green
property float blue
end_header
]])
   pid = 1
   for i = 1,npts do
      pt = d[i]
      objf:write(string.format("%f %f %f %f %f %f %f %f %f\n",
                               pt[1],pt[2],pt[3],
                               pt[4],pt[5],pt[6],
                               pt[7],pt[8],pt[9]))
   end
   objf:close()
end

save_js_for_luma()
-- save_ply_for_potree()
