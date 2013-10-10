io = require 'io'
src_dir = 'arcs/temporary-circle-6132/source/po_scan/a/001/'
wrk_dir = src_dir:gsub("source","work")

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000
tree_resolution = 15

pcfile = src_dir .. 'sweep.xyz'
rawfile = src_dir .. 'sweep.raw'

_G.pc   = PointCloud.PointCloud.new(pcfile, 25000)
_G.normals = pc:get_normal_map()


imageglob = "[0-9]*png"

_G.aligner = model.SweepImageAligner.new(wrk_dir.."/"..imageglob)

-- get azimuth and elevation
azimuth_per_line = 0
azimuth_per_point = 0
elevation_per_point = 0

file = io.open(rawfile, 'r')
line = file:read()
key,val = line:match("^([^0-9-]): ([^%s]*)")
while (key) do
   print(key,val)
   -- this is inverted on purpose. Gets switched later [See: switcheroo]
   if (key == "h") then
      width = tonumber(val)
   elseif (key == "w") then
      height = tonumber(val)
   elseif (key == "azimuth_per_line") then
      azimuth_per_line = tonumber(val)
   elseif (key == "azimuth_per_point") then
      azimuth_per_point = tonumber(val)
   elseif (key == "elevation_per_point") then
      elevation_per_point = tonumber(val)
   end
   line = file:read()
   key,val = line:match("^([^0-9-][^%s]*): ([^%s]*)")
end
file:close()

input_hfov = azimuth_per_line*pc.width
vert_elev  = math.sqrt(elevation_per_point^2 - azimuth_per_point^2)
print("vert_elev", vert_elev)
input_vfov = vert_elev*pc.height

_G.proj_from  = projection.SphericalProjection.new(pc.width,pc.height,input_hfov,input_vfov)
_G.proj_to    = aligner:get_output_projection()
_G.mapper     = projection.Remap.new(proj_from,proj_to)

_G.normals_remap = mapper:remap(normals)
_G.img = aligner:generate_panorama()

image.display{image={normals_remap,img},nrow=1}

scanner_pose = torch.zeros(3)

if nil then
   for _,tree_resolution in pairs({50,25,12,6,1}) do
      _G.tree = octomap.Tree.new(tree_resolution)

      print("loading points")
      tree:add_points(points,scanner_pose,max_radius)

      tree:stats()

      _G.tree_points = tree:get_occupied()

      bbx   = tree:bbx()
      pm,_  = points:min(1):squeeze()
      px,_  = points:max(1):squeeze()
      tpm,_ = tree_points:min(1):squeeze()
      tpx,_ = tree_points:max(1):squeeze()

      output_filename = wrk_dir .. "voxel_grid_"..this_resolution.."mm.xyz"
      print("writing ".. output_filename)
      tree:writePointsXYZ(output_filename)
   end
end
