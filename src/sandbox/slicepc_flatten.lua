require 'gnuplot'
hough = require '../image/hough'

scale = 0.01
pc = PointCloud.PointCloud;
_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/scans0/92.xyz')
--_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/scans0/clean/92_93_94.xyz')
--_G.faro_points = pc.new('/Users/aditya/Work/code/Floored/data/pc/forAditya/scan_naproom_03.xyz')

--faro_points = faro_points:downsample(0.005)

faro_points:make_flattened_images(scale)
z1 = hough.get_hough_transform(faro_points.imagez[1], 1000, 360*4)

yi1 = hough.find_best_lines(z1, 1)[1][2]

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

y, i = torch.sort(faro_points.points:select(2, 3), 1);
_G.sorted_face_centers = faro_points.points:index(1, i);

--sorted_face_centers = geom.quaternion.translate_rotate(translation, quat, sorted_face_centers)
faro_points.points = geom.quaternion.translate_rotate(translation, quat, faro_points.points)

max_z = torch.max(sorted_face_centers, 1):select(2,3)[1];
min_z = torch.min(sorted_face_centers, 1):select(2,3)[1];

max_x = torch.max(sorted_face_centers, 1):select(2,1)[1];
min_x = torch.min(sorted_face_centers, 1):select(2,1)[1];

max_y = torch.max(sorted_face_centers, 1):select(2,2)[1];
min_y = torch.min(sorted_face_centers, 1):select(2,2)[1];

num_points = faro_points.points:size(1);

_G.hist = gnuplot.histc(sorted_face_centers:select(2,3),80);

_G.ycum = torch.cumsum(hist.raw, 1)
_G.yh, _G.ih = torch.sort(hist.raw, 1, true);

histmax, histmaxind = hist.raw:max();
histimg = hist.raw:resize(1, 80)
kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)
_G.dominance = image.convolve(histimg, kernel, 'same'):resize(80)/histmax;

_G.num_slices = torch.sum(torch.gt(dominance, 0.3));

_G.ih = ih:narrow(1,1,num_slices);
_G.ih = torch.sort(ih);
ih:resize(ih:size(1) + 1)
ih[ih:size()] = 79;

for num_slice = 1, num_slices do
	print("  Saving slice " .. num_slice)
	if ih[num_slice + 1] - 1 > ih[num_slice] + 1 then
		local size = ycum[ih[num_slice + 1] - 1] - ycum[ih[num_slice] + 1]
		print("  From ".. ycum[ih[num_slice] + 1] .. " to " .. ycum[ih[num_slice + 1] - 1]);
		--faro_points.points = sorted_face_centers:narrow(1, ycum[ih[num_slice] + 1], size);
		--faro_points.count = size
		local mask = torch.Tensor(faro_points.height, faro_points.width):zero()
		
		for num_point = ycum[ih[num_slice] + 1], ycum[ih[num_slice + 1] - 1] do
			local h = faro_points.hwindices[i[num_point]][1]
			local w = faro_points.hwindices[i[num_point]][2]
			mask[h][w] = 1
		end
		
		faro_points:make_flattened_images(scale, mask)

		filename = 'slices/slice' .. num_slice .. '.png'
	
		image.save(filename, faro_points.imagez)
	end
end
