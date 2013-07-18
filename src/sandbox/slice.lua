--require 'gnuplot'

posefile = "../../../data/96_spring_kitchen/raw_scan/scanner371_job286000_texture_info.txt"
objfile  = "../../../data/96_spring_kitchen/raw_scan/scanner371_job286000.obj"

function is_wall_poly(p)
	return math.abs(p[3]) < 0.05
end

function normalized_angle(norm)
	if norm[1] >= 0 and norm[2] >= 0 then
		x = norm[1]
		y = norm[2]
	elseif norm[1] <= 0 and norm[2] <= 0 then
		x = -norm[1]
		y = -norm[2]
	elseif norm[1] < 0 then
		x = norm[2]
		y = -norm[1]
	elseif norm[2] < 0 then
		x = -norm[2]
		y = norm[1]
	end

	if x == 0 then
		angle = 0
	else
		angle = math.floor(math.deg(math.atan(y/x)));
	end

	if angle == 90 then 
		angle = 0
	end
	
	return angle
end

function rotation_from_buckets(buckets)
	--[[Find the biggest bucket (indicating the greatest 
	number of aligned faces), and calculate the +/- radians 
	to rotate the object to align it with the x or y axis.
	]]
	num, angle = torch.max(buckets, 1);
	return math.rad(angle[1])
end

print "== Loading Scan =="
scan = model.mp.scan(posefile, objfile);
print "== Loading Model Data =="
obj = scan:get_model_data();

-- select subset of 499939 data points
num_small_face_centers = 499939;
small_face_centers = obj.face_centers:narrow(1, 1, num_small_face_centers);
y, i = torch.sort(small_face_centers:select(2, 3), 1);
sorted_face_centers = small_face_centers:index(1, i);

--[[ select all data points
y, i = torch.sort(obj.face_centers:select(2, 3), 1);
z = obj.face_centers:index(1, i);
--]]

max_z = torch.max(sorted_face_centers, 1):select(2,3)[1];
min_z = torch.min(sorted_face_centers, 1):select(2,3)[1];

max_x = torch.max(sorted_face_centers, 1):select(2,1)[1];
min_x = torch.min(sorted_face_centers, 1):select(2,1)[1];

max_y = torch.max(sorted_face_centers, 1):select(2,2)[1];
min_y = torch.min(sorted_face_centers, 1):select(2,2)[1];

width = 3000;
height = 3000;
padding = 200;
scale = 200;

print "== Aligning to Axes =="

buckets = torch.zeros(90);

for num_face = 1, obj.n_faces do
	if is_wall_poly(obj.face_normals[num_face]) then
		local bucket = normalized_angle(obj.face_normals[num_face]);
		buckets[bucket+1] = buckets[bucket+1] + 1
	end
end

euler_angles = torch.Tensor(3);
euler_angles[1] = 0;
euler_angles[2] = -1*rotation_from_buckets(buckets);
euler_angles[3] = 0;

quat = geom.quaternion.from_euler_angle(euler_angles);

translation = torch.Tensor(3);
translation[1] = -1*(max_x + min_x)/2
translation[2] = -1*(max_y + min_y)/2
translation[3] = 0;

aligned_sorted_face_centers = geom.quaternion.translate_rotate(translation, quat, sorted_face_centers)

print "== Scaling =="

max_z = torch.max(aligned_sorted_face_centers, 1):select(2,3)[1];
min_z = torch.min(aligned_sorted_face_centers, 1):select(2,3)[1];

max_x = torch.max(aligned_sorted_face_centers, 1):select(2,1)[1];
min_x = torch.min(aligned_sorted_face_centers, 1):select(2,1)[1];

max_y = torch.max(aligned_sorted_face_centers, 1):select(2,2)[1];
min_y = torch.min(aligned_sorted_face_centers, 1):select(2,2)[1];

scale_x = math.min((width/2 - padding)/max_x, (padding-width/2)/min_x);
scale_y = math.min((height/2 - padding)/max_y, (padding-height/2)/min_y);

scale = math.min(scale_x, scale_y);

print "== Generating Slices =="

slice = torch.Tensor(width, height);

for num_slice = 1, 10 do
	slice = torch.zeros(width, height);
	local slice_z = aligned_sorted_face_centers:narrow(1, (num_slice-1) * num_small_face_centers/10 + 1, num_small_face_centers/10);
	for point_idx = 1,num_small_face_centers/10 do 
		local x = torch.ceil(width/2 + scale * slice_z[point_idx][1]);
		local y = torch.ceil(height/2 + scale * slice_z[point_idx][2]);
		if x > 0 and x < width and y > 0 and y < height then
			slice[x][y] = 1.0;
		end
	end

	filename = 'slices/slice' .. num_slice .. '.png'

	print("  Saving slice " .. num_slice)
	image.save(filename, slice);
end

hist = torch.histc(aligned_sorted_face_centers:select(2,3),10);

--gnuplot.histc(aligned_sorted_face_centers:select(2,3))