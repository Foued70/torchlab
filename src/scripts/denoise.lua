-- implementation of Color Block Matching 3D (CBM3D) from :
-- 
-- A case for denoising before demosaicking color filter array data
-- Sung Hee Park, Hyung Suk Kim, Steven Lansel, Manu Parmar, and Brian
-- A. Wandell

Class()

img = image.load('../data/test/96_spring_kitchen/sweep_1/DSC_0180.jpg')

imglab = image.rgb2lab(img)

block_size = 5 -- 9
search_size = 32 -- 256

-- only do block matching in luminance channel
lum = imglab[1]

-- TODO : reuse low level code from texture fill

