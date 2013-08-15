libopencv = require './libopencv'

opencv    = require './init'

-- use graphics magick to load the image
img = image.load("DSC_0130.png","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
mat = opencv.Mat.fromTensor(img[1])

kptbl = {}
nptbl = {}

-- test all Detector types
for _,dtype in pairs(opencv.Detector.types) do 

   print("--- " .. dtype)
   detector    = opencv.Detector.create(dtype)
   print("made detector")
   kpts, npts  = opencv.Detector.detect(detector,mat,25)
   print("detected " .. npts .. " pts")
   -- save the kpts
   table.insert(kptbl,kpts)
   table.insert(nptbl,npts)
   
end

-- now show that the memory is still around
for ti,kpts in pairs(kptbl) do 
   dtype = opencv.Detector.types[ti]
   npts = nptbl[ti]

   print("------------ "..dtype.." ------------------")

   -- debug draw in opencv
   draw_img = opencv.Mat.fromTensor(img[1]:clone())
   libopencv.debug_keypoints(draw_img,kpts,npts)
   -- libopencv.C.Mat_showImage(draw_img, dtype)
   drawn_t = opencv.Mat.toTensor(draw_img)
   drawn_t = drawn_t:transpose(3,1):transpose(2,3)
   image.display(drawn_t)
   print(drawn_t:size())
   for i = 0,npts-1 do 
      cvp = kpts[i]
      print(i,cvp.pt.x, cvp.pt.y, cvp.response)
   end
end

