log.tic()
if not inFile then 
   imgfile =  CLOUDLAB_SRC .. "/gm/test/lena.jpg"
end
if not outFile then 
   outFile = "test_lumninance_only.png"
end

-- notice graphicsmagic can load NEF files using dcraw !
inImg = gm.Image.new(imgfile)
log.trace("Loaded",imgfile,"in",log.toc())

-- print size gm.Image is not a tensor but a Class there are extra functions
log.trace("inSize",inImg:size())

-- the pointer to image data is different from the tensor, when
-- requesting the tensor we can change colorspace ('LAB') and state
-- how we want the dimentions DHW as in torch, WHD as many other
-- places.

log.tic()
inTensor = inImg:toTensor('float','LAB','DHW')
log.trace("toTensor in",log.toc())

log.trace("torchSize",inTensor:size())

-- make a gray scale image from luminance channel
log.tic()
outImg = gm.Image.new(inTensor[1])
log.trace("outImg in",log.toc())

log.tic()
outImg:save(outFile)
log.trace("Saved",outFile,"in",log.toc())
