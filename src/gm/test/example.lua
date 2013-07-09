log.tic()
-- notice graphicsmagic can load NEF files using dcraw !
inImg = gm.Image.new("test.jpg")
log.trace("Loading in",log.toc())

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

outImg:save("test_luminance_only.png")
log.trace("Save in",log.toc())
