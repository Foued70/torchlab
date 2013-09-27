function [outimg]=DecomposeImage(input_filename, output_filename)
img = imread(input_filename);
if nargin < 2 
  output_filename = "periodic_image.png"
end
imgd = zeros(size(img));
imgd(:) = img;
imgd *= 1/255;

minr = min(min(imgd(:,:,1)));
ming = min(min(imgd(:,:,2)));
minb = min(min(imgd(:,:,3)));

maxr = max(max(imgd(:,:,1)));
maxg = max(max(imgd(:,:,2)));
maxb = max(max(imgd(:,:,3)));

outimg        = zeros(size(img));
outimg(:,:,1) = PeriodicPlusSmoothDecomposition(imgd(:,:,1));
outimg(:,:,2) = PeriodicPlusSmoothDecomposition(imgd(:,:,2));
outimg(:,:,3) = PeriodicPlusSmoothDecomposition(imgd(:,:,3));

imwrite(outimg,output_filename);
