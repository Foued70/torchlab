libopencv = require './libopencv.lua'

img = image.load("DSC_0130.png","byte",nil,"LAB","DHW")

-- TODO: check userdata type
img_cvmat = libopencv.fromTensor(img[1])

for _,dtype in pairs(libopencv.detector_type) do 
   print("------------ "..dtype.." ------------------")
   kptsvect  = libopencv.detect(img_cvmat,dtype)
   -- keep top 25
   kptsvect.length = 25
   -- debug draw in opencv
   draw_img = libopencv.fromTensor(img[1]:clone())
   libopencv.draw_keypoints(draw_img,kptsvect)

   print(kptsvect)
   
   npts = kptsvect.length
   kpts = kptsvect.data
   
   for i = 0,npts do 
      cvp = kpts[i]
      print(i,cvp.pt.x, cvp.pt.y, cvp.response)
   end
end

-- opencv.toTensor(cvpoints)

-- cvpoints = libopencv.detectfeatures(cvmat,"HARRIS")
-- cvpoints = libopencv.detectfeatures(cvmat,"GFTT")
