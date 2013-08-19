
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
for _,dtype in pairs(opencv.Detector.types) do 
   print("--- " .. dtype)
   detector    = opencv.Detector.create(dtype)
   print("made detector")
   kpts, npts  = opencv.Detector.detect(detector,mat_gray.mat,max_points)
   print("detected " .. npts .. " pts")
   -- save the kpts
   table.insert(kptbl,kpts)
   table.insert(nptbl,npts)
end

-- now show that the memory is still around
for ti,kpts in pairs(kptbl) do 
   dtype = opencv.Detector.types[ti]
   npts = nptbl[ti]

   print("------------ "..dtype.." ("..npts..") ------------------")

   -- debug draw in opencv
   draw_img = opencv.Mat.new(bgr:clone())

   opencv.C.dump_keypoints(kpts,npts)
   
   opencv.C.draw_keypoints(draw_img.mat,kpts,npts)

   draw_img:display(dtype..'_BGR')
   
   -- back to torch. display image in torch
   -- make the color channels in torch order
   img_cvt = opencv.ImgProc.convert(draw_img,"BGR2RGB")
   cvt_th  = img_cvt:toTensor("DHW")
   image.display(cvt_th)

   -- print keypoints in torch
   for i = 0,npts-1 do 
      cvp = kpts[i]
      print(i,cvp.pt.x, cvp.pt.y, cvp.response)
   end
end

