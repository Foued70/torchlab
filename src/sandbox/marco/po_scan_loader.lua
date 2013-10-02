src_dir = 'arc/source/po_scan/a/001/'
wrk_dir = 'arc/work/po_scan/a/001/'

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000 
tree_resolution = 15

_G.pc = PointCloud.PointCloud.new(src_dir .. 'sweep.xyz', 25000)


scanner_pose = torch.zeros(3)

i = 1
points = pc.points:clone():mul(i)
this_max_radius = max_radius * i

for _,tree_resolution in pairs({50,25,12,6,1}) do 
   this_resolution = tree_resolution * i
   _G.tree = octomap.Tree.new(this_resolution)
   
   print("loading points")
   tree:add_points(points,scanner_pose,this_max_radius)
   

   tree:stats()

   _G.tree_points = tree:get_occupied()

   bbx   = tree:bbx()
   pm,_  = points:min(1):squeeze()
   px,_  = points:max(1):squeeze()
   tpm,_ = tree_points:min(1):squeeze()
   tpx,_ = tree_points:max(1):squeeze()

   print(points)
   print(tree_points)
   print(bbx)

   printf("X: min: %f, %f max : %f %f (pts,tree)",pm[1],tpm[1],px[1],tpx[1])
   printf("Y: min: %f, %f max : %f %f (pts,tree)",pm[2],tpm[2],px[2],tpx[2])
   printf("Z: min: %f, %f max : %f %f (pts,tree)",pm[3],tpm[3],px[3],tpx[3])
   
   output_filename = wrk_dir .. "voxel_grid_"..this_resolution.."mm.xyz"
   print("writing ".. output_filename)
   tree:writePointsXYZ(output_filename)
end
