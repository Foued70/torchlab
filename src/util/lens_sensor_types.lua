-- Store the highest level information we know about the lens + sensor
-- combinations we are using when capturing higher quality textures
-- with DSLR

local lens_types = {
   nikon_D800E_w18mm = {

      name     = "Nikon D800E with 18mm",

      -- copied from exif info
      sensor_w = 35.9, -- mm
      sensor_h = 24.0, -- mm
      focal    = 18, -- mm

      lens_type = "rectilinear",

   },

   nikon_D5100_w10p5mm = {

      name     = "Nikon D5100 with 10.5mm",

      -- copied from exif info
      sensor_w = 23.6, -- mm
      sensor_h = 15.6, -- mm

      -- from http://michel.thoby.free.fr/Blur_Panorama/Nikkor10-5mm_or_Sigma8mm/Sigma_or_Nikkor/Comparison_Short_Version_Eng.html

      -- From my own experimental measurement, the mapping function of
      -- the Nikkor 10,5mm AF DX f/2.8 is R = 1.47 x f x sin (0.713 x
      -- Omega). The measured focal length is f = 10.58mm. This may be
      -- the farthest from the theoretical formula amongst all
      -- commercially available circular fisheye lenses that have been
      -- tested so far. It is therefore nearly abusive to put it in the
      -- so-called Equi-Solid Angle projection class.

      focal    = 10.58, -- mm

      lens_type = "thoby"
   },

   nikon_10p5mm_calibrated = {

      name     = "Calibrated Nikon D5100 with 10.5mm",

      -- copied from exif info
      sensor_w = 23.6, -- mm
      sensor_h = 15.6, -- mm

      focal    = 10.58, -- mm

      -- calibration parameters from Davide Scaramuzza's OCamCalib_v2.0
      lens_type = "scaramuzza",
      pol = torch.Tensor({
                            -525.9304775020237,
                            0,
                            0.0006587985106851861,
                            9.8724309479257e-08,
                            1.814892907921985e-10,
                         }),
      -- this is in pixel coordinates in an image we had to rescale to
      -- get Scaramuzza's code to run. So we need to revert to
      -- normalized coordinates.
      invpol = torch.Tensor({
                               -1.406840268367202,
                               -16.61904995462069,
                               -79.52037729412098,
                               -192.6306842596766,
                               -239.1074046506864,
                               -132.7044023233921,
                               -35.37184841197575,
                               473.1309124770347,
                               784.2197736505799
                            }),
      
      cal_width = 1200,
      cal_height = 795,
      -- for sanity's sake reverse the names x <-> y 
      cal_yc = 401.79351104,
      cal_xc = 602.2562304
   }

}

return lens_types

