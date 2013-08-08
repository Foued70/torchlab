opencv    = opencv.init
libopencv = require './libopencv.lua'

img = image.load("DSC_0130.png","byte",nil,"LAB","DHW")

mat = opencv.MatfromTensor(img[1])

kptbl = {}
nptbl = {}

for _,dtype in pairs(libopencv.detector_type) do 

   kpts, npts  = libopencv.detect(mat,dtype,25)

   -- save the kpts
   table.insert(kptbl,kpts)
   table.insert(nptbl,npts)
   
end

-- now show that the memory is still around
for ti,kpts in pairs(kptbl) do 
   dtype = libopencv.detector_type[ti]
   npts = nptbl[ti]

   print("------------ "..dtype.." ------------------")

   -- debug draw in opencv
   draw_img = opencv.MatfromTensor(img[1]:clone())
   libopencv.debug_keypoints(draw_img,kpts,npts)
   -- libopencv.C.Mat_showImage(draw_img, dtype)
   drawn_t = opencv.MattoTensor(draw_img)
   drawn_t = drawn_t:transpose(3,1):transpose(2,3)
   image.display(drawn_t)
   print(drawn_t:size())
   for i = 0,npts-1 do 
      cvp = kpts[i]
      print(i,cvp.pt.x, cvp.pt.y, cvp.response)
   end
end

