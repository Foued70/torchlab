-- use graphics magick to load the image
img_src = image.load(CLOUDLAB_SRC.."/image/test/baboon200.jpg","byte",nil,"LAB","DHW")

t = opencv.imgproc.getDefaultStructuringMat(5);
t:toTensor()

types = require '../types/Morph.lua'
--note the center is based on 0 indexing
t2 = opencv.imgproc.getStructuringElement(types.MORPH_CROSS, 25, 25, 8, 8);
print(t2:toTensor())

--libopencv.C.Mat_convert(mat, mat,libopencv.cvtColor_types.RGB2GRAY);
--note that this overrides what is in mat, which points to the same memory as t_torch2
dilated = t2:clone();
opencv.imgproc.dilate(dilated, t);
print(dilated:toTensor())

eroded = t2:clone();
opencv.imgproc.erode(eroded, t);
print(eroded:toTensor())


-- opencv conversions
