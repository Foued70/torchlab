hough = require '../image/hough'

scale = 0.0150
radius = 7

for i=189,195 do

	fname1 = '/Users/aditya/Work/code/Floored/data/pc/scans0/'..i..'.xyz'
	print('Source: '.. fname1);

	pc1 = PointCloud.PointCloud;
	faro_points1 = pc1.new(fname1, radius)

	print "flattening point cloud .."
	faro_points1:make_flattened_images(scale)

	print "saving images .."
	image.save('flattened/'..i..'.png', faro_points1.imagez)
	
	pc1 = nil;
	collectgarbage();

end