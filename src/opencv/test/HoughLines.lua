opencv    = require '../init'

-- use graphics magick to load the image
img_src = image.load(CLOUDLAB_SRC.."/image/test/baboon200.jpg","byte",nil,"LAB","DHW")
matMat_src = opencv.Mat.new(img_src[1])

dst = opencv.ImgProc.CannyDetectEdges(matMat_src.mat, 50, 200)
print(dst:toTensor())
lines = opencv.ImgProc.HoughLinesRegular(dst.mat, 1, math.pi/180, 100, 0, 0 );
print(lines:toTensor())
linesP = opencv.ImgProc.HoughLinesProbabilistic(dst.mat, 1, math.pi/180, 50, 50, 10 );
print(linesP:toTensor())
