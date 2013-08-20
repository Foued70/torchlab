
opencv    = require '../init'

img_path = CLOUDLAB_SRC.."/image/test/lena.jpg"
max_points = 150
-- use graphics magick to load the image
wand = image.Wand.new(img_path)
gray = wand:toTensor("byte","G","HWD"):squeeze()

-- unlike torch, opencv uses BGR and HWD
bgr  = wand:toTensor("byte","BGR","HWD")

-- convert 1st channel of torch tensor to opencv matrix
mat_gray = opencv.Mat.new(gray)
-- store color version for displaying keypoints
mat_rgb  = opencv.Mat.new(img_path)

kptbl = {}
nptbl = {}

-- test all Detector types
for dtype,val in pairs(opencv.Detector.types) do 
   print("--- " .. dtype)
   detector    = opencv.Detector.new(dtype)
   print("made detector")

   detector:parameters()

   kpts, npts  = detector:detect(mat_gray,max_points)
   print("detected " .. npts .. " pts")
   -- save the kpts
   kptbl[val] = kpts
   nptbl[val] = npts
end

-- now show that the memory is still around
for dtype,val in pairs(opencv.Detector.types) do 
   npts = nptbl[val]
   kpts = kptbl[val]

   print("------------ "..dtype.." ("..npts..") ------------------")

   -- debug draw in opencv
   draw_img = opencv.Mat.new(bgr:clone())

   opencv.utils.draw_keypoints(draw_img,kpts,npts)

   draw_img:display(dtype..'_BGR')
   
   -- back to torch. display image in torch
   -- make the color channels in torch order
   img_cvt = opencv.Mat.new()
   draw_img:convert(img_cvt,"BGR2RGB")
   cvt_th  = img_cvt:toTensor("DHW")
   image.display(cvt_th)

   -- print keypoints in torch
   if false then 
      for i = 0,npts-1 do 
         cvp = kpts[i]
         print(i,cvp.pt.x, cvp.pt.y, cvp.response)
      end
   end
end

