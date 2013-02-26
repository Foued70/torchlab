require 'image'

local util = require 'util'
local geom = util.geom

local pi = math.pi
local piover2 = math.pi * 0.5 
local r2d = 180 / pi
local d2r = pi / 180

-- image.rotate does not do what we want
function img_rot (img)
   return  image.vflip(img:transpose(2,3))
end

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir',
           'images/',
           'directory with the images to load')
cmd:option('-outdir','output/')
cmd:text()

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.imagesdir
outdir     = params.outdir .. "/"


-- load images
if not images then
   images = {}
   imgfiles = paths.files(imagesdir)
   imgfiles()
   imgfiles()
   local cnt = 1
   for f in imgfiles do
      if f == ".DS_Store" then -- exclude OS X automatically-created backup files 
         printf("--- Skipping .DS_Store file")
      else
         local imgfile = imagesdir.."/"..f
         printf("Loading : %s", imgfile)
         images[cnt] = image.load(imgfile)
         cnt = cnt + 1
      end
   end
end

-- start of camera object (extension of pose object...)

-- FIXME write .ini loader...

nikon_D800E_w18mm = {
      name     = "Nikon D800E with 18mm",
      -- copied from exif info
      sensor_w = 35.9, -- mm
      sensor_h = 24.0, -- mm
      focal    = 18, -- mm

      -- copied from .ini computed in hugin
      a       =  0.0762999,
      b       = -0.167213,
      c       =  0.061329
}

nikon_D5100_w10p5mm = {
      name     = "Nikon D5100 with 10.5mm",
      -- copied from exif info
      sensor_w = 23.6, -- mm
      sensor_h = 15.6, -- mm
      focal    = 10.5, -- mm

      -- computed with opencv
      a        = -0.3889126043367629,
      b        =  0.1473722957858515,
      c        = -0.02577248837247648,
      p_one    =  0.001583298478238885,
      p_two    =  0.01061379290496793
   }

camera = nikon_D5100_w10p5mm


-- add image data to camera

function update_imagedata(camera,image)

   local w = image:size(3)
   local h = image:size(2)

   camera.image_w  = w -- px
   camera.image_h  = h -- px

   -- same as in pose
   camera.center_x = w*0.5
   camera.center_y = h*0.5

   camera.sensor_diag  = 0.5*math.sqrt(camera.sensor_w*camera.sensor_w + 
                                       camera.sensor_h*camera.sensor_h)

   camera.px_per_mm_x = w / camera.sensor_w 
   camera.px_per_mm_y = h / camera.sensor_h 

   camera.fovw     = torch.atan2((camera.sensor_w*0.5),camera.focal) -- rad
   camera.fovh     = torch.atan2((camera.sensor_h*0.5),camera.focal) -- rad

   camera.degree_per_px_x = (r2d * camera.fovw)/w
   camera.degree_per_px_y = (r2d * camera.fovh)/h

   -- focal length in pixels
   camera.focal_px = camera.focal * w / camera.sensor_w

   camera.aspect_ration = h / w 

   return camera

end

-- creates an (equidistant) map from a unit sphere to a camera image either:
-- 
--  + rectilinear (perspective) : r = f * tan(theta) 
--  + stereographic             : r = 2 * f * tan(theta/2)
--  + orthographic              : r = f * sin(theta)
--  + equisolid                 : r = 2 * f * sin(theta/2)
--  + thoby (fisheye)           : r = k1 * f * sin(k2*theta)
--  + eqidistant                : r = f * theta  (for unit sphere f = 1)

function projection_to_sphere (camera, projection,scale,debug)

   if not projection then
      projection = "rectilinear"
   end
   if not scale then
      scale = 1
   end
   local imgw = camera.image_w
   local imgh = camera.image_h
   local hlfw = camera.center_x
   local hlfh = camera.center_y

   local fovw = camera.fovw
   local fovh = camera.fovh

   local mapw = imgw * scale
   local maph = imgh * scale
   -- map has to be bigger than the input image, pick piover2 as an upper bound.
   local map_fovw = piover2
   local map_fovh = piover2 * (imgh / imgw)
   local max_imgw = hlfw
   local max_imgh = hlfh

   local xrow = torch.linspace(-map_fovw,map_fovw,mapw)
   local ycol = torch.linspace(-map_fovh,map_fovh,maph)
   
   -- pythagorean theorem on a unit sphere is cos(c) = cos(a)cos(b)
   local r_map = xrow:repeatTensor(maph,1):cos() -- cos(a)
   r_map:cmul(ycol:repeatTensor(mapw,1):t():cos()) -- * cos(b)
   r_map:acos() -- c = arccos(cos(a)cos(b)

   -- find the ratio r' / r
   local ratio_map = r_map:clone() -- copy

   if (projection == "rectilinear") then
      -- rectilinear : (1/f) * r' = tan(theta) 
      ratio_map:tan()
      max_imgw = camera.focal_px * torch.tan(map_fovw)
      max_imgh = camera.focal_px * torch.tan(map_fovh)
   elseif (projection == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      local k1 = 1.47
      local k2 = 0.713
      ratio_map:mul(k2):sin():mul(k1)
      max_imgw = k1 * camera.focal_px * torch.sin(k2*map_fovw)
      max_imgh = k1 * camera.focal_px * torch.sin(k2*map_fovh)
   elseif (projection == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      ratio_map:mul(0.5):tan():mul(2)
      max_imgw = 2 * camera.focal_px * torch.tan(0.5*map_fovw)
      max_imgh = 2 * camera.focal_px * torch.tan(0.5*map_fovh)
   elseif (projection == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      ratio_map:sin()
      max_imgw = torch.sin(map_fovw)
      max_imgh = torch.sin(map_fovh)
   elseif (projection == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      ratio_map:mul(0.5):sin():mul(2)
      max_imgw = 2 * camera.focal_px * torch.sin(0.5*map_fovw)
      max_imgh = 2 * camera.focal_px * torch.sin(0.5*map_fovh)
   else
      print("ERROR don't understand projection")
      return nil
   end

   -- ratio (in unit sphere coordinates)
   ratio_map:cdiv(r_map)           -- ratio = r' / r

   -- *** check this ****
   -- x,y index (map unit sphere to pixel coords)
   -- torch indexes from 1,size_x
   local xval = hlfw - 0.5 
   local yval = hlfh - 0.5 
   -- start with old x and y values centered at zero
   local xindex = torch.linspace(-max_imgw,max_imgw,mapw)
   local yindex = torch.linspace(-max_imgh,max_imgh,maph)

   local xmap = xindex:repeatTensor(maph,1)
   xmap:cmul(ratio_map):add(hlfw + 0.5)
   local mask = xmap:gt(1) + xmap:lt(imgw)

   -- ymap is offset by 1 to multiply by stride
   local ymap = yindex:repeatTensor(mapw,1):t():contiguous()
   ymap:cmul(ratio_map):add(hlfh + 0.5)

   mask = mask + ymap:gt(1) + ymap:lt(imgh)

   -- reset mask to 0 and 1 (valid parts of mask must pass all 4 tests)
   mask[mask:ne(4)] = 1
   mask[mask:eq(4)] = 0
   printf("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())
   
   -- keep everything inbounds so no if checks in inner loops
   ymap[mask] = 1
   xmap[mask] = 1

   -- copy rows into 1D map (index = x + ((y - 1)* stride))
   -- CAREFUL must floor before multiplying by stride or does not make sense.
   -- -0.5 then floor is equivalient to adding 0.5 -> floor -> -1 before multiply by stride.
   local outmap = ymap:clone():add(-0.5):floor()
   -- ymap -1 so that multiply by stride makes sense (imgw is the stride)
   -- to map (1,1) ::  y = 0  * stride + x = 1 ==> 1
   -- to map (2,1) ::  y = 1  * stride + x = 1 ==> stride + 1 etc. 
   
   outmap:mul(imgw):add(xmap + 0.5)

   -- remove spurious out of bounds from output
   outmap[mask] = 1 

   local index_map = outmap:long() -- round (+0.5 above) and floor

   index_map:resize(index_map:size(1)*index_map:size(2))
   if debug then
      printf(" x map from %d to %d (max: %d)",xmap:min(),xmap:max(),imgw)
      printf(" y map from %d to %d (max: %d)",ymap:min(),ymap:max(),imgh)
      printf("1D map from %d to %d (max: %d)",index_map:min(),index_map:max(),index_map:size(1))
   end

   return index_map,fovw,fovh,mask,mapw,maph

end

function sphere_to_projection()

end

-- image is 3 x map dims
-- now map is 1D (this is faster) 
function remap(img, map, map_w, map_h, map_mask)
   local out = torch.Tensor()
   -- fast 1D remap
   if (map:nDimension() ~= 1) then
      print("ERROR map should be 1D Tensor")
      return nil
   end

   local ndim     = img:nDimension()
      
   if (ndim == 2) then
      -- single channel 2D

      local imgh = img:size(1)
      local imgw = img:size(2)

      img:resize(imgh * imgw)
      out = img[map] -- uses new indexing feature
      out[map_mask] = 0  -- erase out of bounds

      img:resize(imgh,imgw)
      out:resize(map_h,map_w)

   elseif (ndim == 3) then
      -- n channel (RGB) (n x h x w)
      out:resize(img:size(1),map:nElement())
      
      for d = 1,img:size(1) do -- loop through channels
         local imgd = img[d]
         imgd:resize(imgd:size(1)*imgd:size(2))

         out[d] = imgd[map]
         out[d][map_mask] = 0

      end

      out:resize(img:size(1),map_h,map_w)
   end
   return out
end


img = images[1]
update_imagedata(camera,img)

equimap, max_theta_x, max_theta_y, equimask,equiw,equih  = 
   projection_to_sphere(camera,"stereographic",0.5)

sys.tic()
output_image = remap(img,equimap,equiw,equih,equimask)
printf("time fast: %2.4fs",sys.toc())

image.display{image={output_image}}
image.display(img)

test_slow = false

if test_slow then
   if (not xmap or not ymap) then
      print("Error can't test need xmap and ymap which were made local")
   else
      function remap_xy(img,xmap,ymap)
         local out = torch.Tensor(img:size(1),xmap:size(1),xmap:size(2))
         for yi = 1,xmap:size(1) do
            for xi = 1,xmap:size(2) do 
               out[1][yi][xi] = img[1][ymap[yi][xi]][xmap[yi][xi]]
               out[2][yi][xi] = img[2][ymap[yi][xi]][xmap[yi][xi]]
               out[3][yi][xi] = img[3][ymap[yi][xi]][xmap[yi][xi]]
            end
         end
         return out
      end
      
      sys.tic()
      output_image_slow = remap_xy(img,xmap,ymap)
      printf("time slow: %2.4fs",sys.toc())
      print("Testing against slow version")
      img1 = img[1]
      img1f = img[1]
      img1f:resize(img1:size(1)*img1:size(2))
      sqmap = equimap:clone()
      sqmap:resize(xmap:size(1),xmap:size(2))
      imgw = img:size(3)
      err = 0
      cnt = 0
      for i = 1,img:size(2),100 do
         for j = 1,img:size(3),100 do 
            cnt = cnt + 1
            local xm = math.floor(xmap[i][j] + 0.5)
            local ym = math.floor(ymap[i][j] + 0.5)
            local im = math.floor((ym - 1)*1024 + xm + 0.5)
            local sq = sqmap[i][j]
            local em = equimap[(i-1)*1024 + j]
            local im1 = img1[ym][xm]
            local im2 = img1f[im]
            local im3 = img1f[sq]
            local dif = im - sq
            if (torch.abs(dif) > 0) then
               printf("[%3d][%3d] x: %4d y: %4d img: %f",
                      i,j, xm,ym, im1)
               printf("   im: %d  sq: %d eq: %d diff: %d img: %f %f",
                      im, sq, em, im - sq, im2, im3) 
               printf(" ERRROR: %d",dif)
               err = err + 1
            end
         end    
      end
      printf("Err: %d/%d",err,cnt)
   end
end
