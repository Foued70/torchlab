require 'gnuplot'
hough = require '../image/hough'

scale = 0.01
pc = PointCloud.PointCloud;
_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/scans0/sweep_001.xyz')
--_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/scans0/clean/92_93_94.xyz')
--_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/forAditya/scan_naproom_03.xyz')

--faro_points = faro_points:downsample(0.005)

faro_points:make_flattened_images(scale)
z1 = hough.get_hough_transform(faro_points.imagez[1], 1000, 360*4)

-- find dominant direction 
yi1 = hough.find_best_lines(z1, 1)[1][2]

-- divide by 4.0 to get degrees out of 360.0
print((yi1-1)/4.0);

euler_angles = torch.Tensor(3);
euler_angles[1] = 0;
euler_angles[2] = math.rad((yi1-1)/4.0);
euler_angles[3] = 0;

quat = geom.quaternion.from_euler_angle(euler_angles);

translation = torch.Tensor(3);
translation[1] = 0--1*(max_x + min_x)/2
translation[2] = 0--1*(max_y + min_y)/2
translation[3] = 0;

-- sort points along the z-direction
y, i = torch.sort(faro_points.points:select(2, 3), 1);
_G.sorted_face_centers = faro_points.points:index(1, i);

-- rotate points
--sorted_face_centers = geom.quaternion.translate_rotate(translation, quat, sorted_face_centers)
faro_points.points = geom.quaternion.translate_rotate(translation, quat, faro_points.points)
faro_points:reset_point_stats()

-- find bounding boxes (currently unused)
max_z = torch.max(sorted_face_centers, 1):select(2,3)[1];
min_z = torch.min(sorted_face_centers, 1):select(2,3)[1];

max_x = torch.max(sorted_face_centers, 1):select(2,1)[1];
min_x = torch.min(sorted_face_centers, 1):select(2,1)[1];

max_y = torch.max(sorted_face_centers, 1):select(2,2)[1];
min_y = torch.min(sorted_face_centers, 1):select(2,2)[1];

num_points = faro_points.points:size(1);

-- create a histogram in z- direction with 80 bins
_G.hist = gnuplot.histc(sorted_face_centers:select(2,3),80);

-- calculate cumulative sum
_G.ycum = torch.cumsum(hist.raw, 1)

-- sort histogram to find top peaks
_G.yh, _G.ih = torch.sort(hist.raw, 1, true);

-- find max and smooth histogram
histmax, histmaxind = hist.raw:max();
histimg = hist.raw:resize(1, 80)
kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)

-- use max to calculate measure of 'relative dominance' of a peak
_G.dominance = image.convolve(histimg, kernel, 'same'):resize(80)/histmax;

-- find optimal number of slices
_G.num_slices = torch.sum(torch.gt(dominance, 0.3));

-- select slices
_G.ih = ih:narrow(1,1,num_slices);
_G.ih = torch.sort(ih);

-- add ceiling (upper bound of slices) -- could be an incorrect hypothesis
ih:resize(ih:size(1) + 1)
ih[ih:size()] = 79;

-- for each slice
for num_slice = 1, num_slices do
	print("  Saving slice " .. num_slice)
	if ih[num_slice + 1] - 1 > ih[num_slice] + 1 then
		-- select sliced points just above the lower bound and just below the upper bound
		local size = ycum[ih[num_slice + 1] - 1] - ycum[ih[num_slice] + 1]
		print("  From ".. ycum[ih[num_slice] + 1] .. " to " .. ycum[ih[num_slice + 1] - 1]);
		print("       ".. sorted_face_centers[ycum[ih[num_slice]]][3] .. " to " .. sorted_face_centers[ycum[ih[num_slice + 1]]][3]);
		--faro_points.points = sorted_face_centers:narrow(1, ycum[ih[num_slice] + 1], size);
		--faro_points.count = size
		-- create mask with required points
		local mask = torch.Tensor(faro_points.height, faro_points.width):zero()
		
		for num_point = ycum[ih[num_slice] + 1], ycum[ih[num_slice + 1] - 1] do
			local h = faro_points.hwindices[i[num_point]][1]
			local w = faro_points.hwindices[i[num_point]][2]
			mask[h][w] = 1
		end
		
		-- make flattened images using mask
		faro_points:make_flattened_images(scale, mask)

		filename = 'slices/slice' .. num_slice .. '.png'
	
		image.save(filename, faro_points.imagez)
	end
end
