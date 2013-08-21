-- use graphics magick to load the image
img_src = image.load(CLOUDLAB_SRC.."/image/test/baboon200.jpg","byte",nil,"LAB","DHW")
matMat_src = opencv.Mat.new(img_src[1])

dst = opencv.imgproc.CannyDetectEdges(matMat_src, 50, 200)
print(dst:toTensor())
lines = opencv.imgproc.HoughLinesRegular(dst, 1, math.pi/180, 100, 0, 0 );
print(lines:toTensor())
linesP = opencv.imgproc.HoughLinesProbabilistic(dst, 1, math.pi/180, 50, 50, 10 );
print(linesP:toTensor())
