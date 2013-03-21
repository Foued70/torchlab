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
   }
}

return lens_types

