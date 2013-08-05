--if (hist1[i] ~= 0 and hist2[i] ~= 0) then
--			score = score + ((hist1[i] - hist2[i])^2) / (hist1[i] + hist2[i]) / 2;
--		end

local function chi_distance(hist1, hist2)
	local score = 0;
	for i = 1, hist1:size(1) do
		score = score + ((hist1[i] - hist2[i])^2) --/ (hist1[i] + hist2[i]) / 2;
	end
	return score
end

hough = require '../image/hough'

scale = 0.005
radius = 7

for i=70,89 do

	pc1 = PointCloud.PointCloud;
	pc2 = PointCloud.PointCloud;

	print "reading point cloud .."
	--faro_points1 = pc1.new('/Users/aditya/Work/code/Floored/data/pc/forAditya/scan_naproom_03.xyz')
	--faro_points2 = pc2.new('/Users/aditya/Work/code/Floored/data/pc/forAditya/scan_naproom_05.xyz')

	--faro_points1 = pc1.new('/Users/aditya/Work/code/Floored/data/pc/scans0/clean/92.xyz')
	--faro_points2 = pc2.new('/Users/aditya/Work/code/Floored/data/pc/scans0/clean/93.xyz')

	--faro_points1 = pc1.new('/Users/aditya/Work/code/Floored/data/pc/scans0/76.xyz', radius)
	--faro_points2 = pc2.new('/Users/aditya/Work/code/Floored/data/pc/scans0/77.xyz', radius)
	
	j = i+1;

	fname1 = '/Users/aditya/Work/code/Floored/data/pc/scans0/'..i..'.xyz'
	fname2 = '/Users/aditya/Work/code/Floored/data/pc/scans0/'..j..'.xyz'
	
	print('Source: '.. fname1);
	print('Target: '.. fname2);

	
	_G.faro_points1 = pc1.new(fname1, radius)
	_G.faro_points2 = pc2.new(fname2, radius)

	--faro_points1 = faro_points1:downsample(0.005)
	--faro_points2 = faro_points2:downsample(0.005)

	print "flattening point cloud .."
	faro_points1:make_flattened_images(scale)
	faro_points2:make_flattened_images(scale)

	print "saving images .."
	image.save('alignment_results/'..i..'_'..j..'_imgz_target.png', faro_points1.imagez)
	image.save('alignment_results/'..i..'_'..j..'_imgz_source.png', faro_points2.imagez)

	_G.z1 = hough.get_hough_transform(faro_points1.imagez[1], 1000, 360)
	_G.z2 = hough.get_hough_transform(faro_points2.imagez[1], 1000, 360)

	--zbl = hough.find_best_lines(z, 4);

	--image.save('imgz_target_hough.png', z1, "I")
	--image.save('imgz_source_hough.png', z2, "I")

	threshval = 0.05;

	z1_nothresh = z1:clone();
	z2_nothresh = z2:clone();

	z1[z1:lt(threshval)] = 0;
	z2[z2:lt(threshval)] = 0;

	sumz1 = torch.sum(z1, 1)
	sumz1_norm = sumz1[1] / torch.sum(sumz1[1]);

	sumz2 = torch.sum(z2, 1)
	sumz2_norm = sumz2[1] / torch.sum(sumz2[1]);

	t = sumz2_norm:repeatTensor(1, 2)[1]
	t_img = z2_nothresh:repeatTensor(1, 2)

	print(t:size());

	mindist = 100000;
	mindist_img = 100000;
	mindistangle = -1;

	for angle = 1, 360 do
		t_rot = t:narrow(1, 1+angle, 360)
	
		dist = chi_distance(sumz1_norm, t_rot);

		if(dist < mindist) then
			mindist = dist
			mindistangle = angle;
		end
	
		t_img_rot = t_img:narrow(2, 1+angle, 360)
	
		dist_img = torch.dist(z1_nothresh, t_img_rot);
	
		print(angle .. " -- 1D: " .. dist .. " 2D: " .. dist_img);
	end

	print(mindistangle);

	euler_angles = torch.Tensor(3);
	euler_angles[1] = 0;
	euler_angles[2] = math.rad(mindistangle - 360);
	euler_angles[3] = 0;

	quat = geom.quaternion.from_euler_angle(euler_angles);

	translation = torch.Tensor(3);
	translation[1] = 0
	translation[2] = 0
	translation[3] = 0;


	faro_points2.points = geom.quaternion.translate_rotate(translation, quat, faro_points2.points)

	faro_points2:reset_point_stats();

	faro_points2:make_flattened_images(scale)

	image.save('alignment_results/'..i..'_'..j..'_imgz_source_rotated.png', faro_points2.imagez)
	
	_G.final_result = torch.zeros(3, math.max(faro_points1.imagez[1]:size(1), faro_points2.imagez[1]:size(1)), math.max(faro_points1.imagez[1]:size(2), faro_points2.imagez[1]:size(2)))
	
	final_result[{1, {1, faro_points1.imagez[1]:size(1)}, {1, faro_points1.imagez[1]:size(2)}}] = faro_points1.imagez[1];
	
	final_result[{2, {1, faro_points2.imagez[1]:size(1)}, {1, faro_points2.imagez[1]:size(2)}}] = faro_points2.imagez[1];
	
	image.save('alignment_results/'..i..'_'..j..'_final_result.png', final_result)


	--threshval = 0.75;

	--z[z:lt(threshval)] = 0;
end