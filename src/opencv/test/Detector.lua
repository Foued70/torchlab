
opencv    = require '../init'

img_path = CLOUDLAB_SRC.."/image/test/lena.jpg"
-- use graphics magick to load the image
_G.wand = image.Wand.new(img_path)
_G.gray = wand:toTensor("byte","G","HWD"):squeeze()
print(gray:min(), gray:max())
-- unlike opencv uses BGR and HWD
_G.bgr  = wand:toTensor("byte","BGR","HWD")
-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
mat_gray = opencv.Mat.new(gray)

mat_rgb = opencv.Mat.new(img_path)
-- mat_graycv = opencv.C.Mat_convert(mat_rgb)
kptbl = {}
nptbl = {}

-- test all Detector types
for _,dtype in pairs(opencv.Detector.types) do 

   print("--- " .. dtype)
   detector    = opencv.Detector.create(dtype)
   print("made detector")
   kpts, npts  = opencv.Detector.detect(detector,mat_gray.mat,25)
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

   opencv.C.Mat_showImage(draw_img.mat,dtype)
   -- back to torch image display
   -- drawn_t = opencv.Mat.toTensor(draw_img)
   -- drawn_t = drawn_t:transpose(3,1):transpose(2,3)

   -- image.display(drawn_t)

   -- print(drawn_t:size())

   for i = 0,npts-1 do 
      cvp = kpts[i]
      print(i,cvp.pt.x, cvp.pt.y, cvp.response)
   end
end

