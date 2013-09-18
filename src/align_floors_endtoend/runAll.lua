path = require 'path'
base_dir = '/Users/stavbraun/Desktop/play/2013_09_08_Office'
base_dir_transformed = '/Users/stavbraun/Desktop/play/2013_09_08_Office/xyz_transf'
base_dir_downsampled= '/Users/stavbraun/Desktop/play/2013_09_08_Office/xyz_down'
util.fs.mkdir_p(base_dir_transformed)
util.fs.mkdir_p(base_dir_downsampled)

all_files = util.fs.files_only(path.join(base_dir, 'XYZ'))
all_files_xyz = {}
for i =1, table.getn(all_files) do
	if path.extname(all_files[i]) == ".xyz" then
		all_files_xyz[table.getn(all_files_xyz)+1] = all_files[i]
	end
end
minWanted = 25
maxWanted = table.getn(all_files_xyz)-1
sweep1 = path.basename(all_files_xyz[minWanted], path.extname(all_files_xyz[maxWanted]))
sweep1_lua = align_floors_endtoend.Sweep.new(base_dir, sweep1)

sweep1_lua:axisAlign()
corners1, flattenedxy1, flattenedv1 = sweep1_lua:flattenAndCorners(true)
image.display(align_floors_endtoend.Sweep.flattened2Image(flattenedxy1))
sweep1_lua:transformIn3D()
--sweep1_lua:getPC():save_global_points_to_xyz(path.join(base_dir_transformed, sweep1 .. ".xyz"))

for i=minWanted, maxWanted do --table.getn(all_files_xyz)-1 do 
	sweep1 = path.basename(all_files_xyz[i], path.extname(all_files_xyz[i]))
	sweep2 = path.basename(all_files_xyz[i+1], path.extname(all_files_xyz[i+1]))
	sweep1_lua = align_floors_endtoend.Sweep.new(base_dir, sweep1)
	sweep2_lua = align_floors_endtoend.Sweep.new(base_dir, sweep2)
	sweep12 = align_floors_endtoend.SweepPair.new(base_dir, sweep2_lua, sweep1_lua)
	sweep12:getAllTransformations()
--	sweep12:setInlierTransformation(1)
	sweep12:setBestDiffTransformation(1)
	sweep12:doIcp2d()
	sweep2_lua:transformIn3D()
	sweep2_lua:getPC():save_global_points_to_xyz(path.join(base_dir_transformed, sweep2 .. ".xyz"))
end

for i=minWanted, maxWanted do --table.getn(all_files_xyz)-1 do 
	sweep1 = path.basename(all_files_xyz[i], path.extname(all_files_xyz[i]))
	sweep1_lua = align_floors_endtoend.Sweep.new(base_dir, sweep1)
	sweep1_lua:getPC():save_downsampled_global_to_xyz(.1, path.join(base_dir_downsampled, sweep1 .. ".xyz"))
end

image.display(combined)



--combine uniformly
rotate_translate = geom.quaternion.rotate_translate

minWanted = 25
maxWanted = 60
pc_total = PointCloud.PointCloud.new()
first = true
leafsize = .01
for i=41, 60 do --table.getn(all_files_xyz)-1 do 
	print(i)
	log.tic()
	sweep1 = path.basename(all_files_xyz[i], path.extname(all_files_xyz[i]))
	sweep1_lua = align_floors_endtoend.Sweep.new(base_dir, sweep1)
	pc = sweep1_lua:getPC();
	if first then
		pc_total.points = pc:get_global_points()
		pc_total.rgb = pc.rgb
		first = false
		pc_total.count = pc.count
	else
		pc_total.count = pc_total.count + pc.count
		pc_total.points = torch.cat(pc_total.points, pc:get_global_points(),1)
		pc_total.rgb = torch.cat(pc_total.rgb, pc.rgb,1)
	end
	pc_total:reset_point_stats()
	local downsampled = pc_total:downsample(leafsize)
   	pc_total.points = downsampled.points
   	pc_total.rgb = downsampled.rgb
   	pc_total.count = downsampled.count
   	collectgarbage()
   	print(pc_total.count)
   	print(log.toc())
end

pc_total:write("combined" .. minWanted .. "_" .. maxWanted .. ".xyz")
