require 'sys'
require 'torch'
require 'math'
require 'image'
require 'paths'

local Pose = require('Pose')

-- this is container class for all the poses
local Poses = torch.class('Poses')


--
-- Email containing information about the pose file from matterport
--
-- From: Florence Shaffer <florence@matterport.com>
-- Date: Tue, Sep 18, 2012 at 3:53 PM
-- The fields that make up each line are:

--     <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
--     <center v> <degrees per px x> <degrees per px y>

-- The first seven numbers define a transform from local sweep
-- coordinates to the global coordinates used for vertices in the .obj.
-- In the local sweep coordinates, the center of the texture is what
-- you'd see looking along the +x axis, and the +z axis is up (towards
-- the top of the texture).  The quaternion defined by the coefficients
-- (qx, qy, qz, qw) is a rotation from local sweep coordinates to
-- global coordinates, and the position (tx, ty, tz) is the origin of
-- the sweep in global coordinates.

-- The next two numbers define the uv coordinates of the point you
-- should treat as the center of the texture.  The u will always be
-- 0.5, but the v will vary a little bit due to how we crop the
-- texture.

-- The final two numbers are change in degrees per pixel as you move
-- around in the image horizintally (x) or vertically (y).

function Poses:__init(posefile)
   if (posefile) then
      sys.tic()
      local posedir = paths.dirname(posefile)
      self.images = {}
      self.names  = {}
      self.nposes =
         tonumber(io.popen(string.format("grep -c '.' %s",posefile)):read())
      log.trace(self.nposes, "poses found in", sys.toc())
      sys.tic()
      self.data = torch.Tensor(self.nposes,19)

      local pf = io.open(posefile)
      local pc = 1
      for pl in pf:lines() do
         local n,pd = pl:match("^([%a%d_.]+) (.*)")
         self.names[pc]    = n
         self.images[pc]   = image.load(string.format("%s/%s", posedir, n))
         self.data[pc][18] = self.images[pc]:size(3) -- width
         self.data[pc][19] = self.images[pc]:size(2) -- height

         local k = 1
         for n in pd:gmatch("[-.%d]+") do
            self.data[pc][k] = tonumber(n)
            k = k + 1
         end
         self.data[pc][16] = self.data[pc][18] *    self.data[pc][8]
         self.data[pc][17] = self.data[pc][19] * (1-self.data[pc][9])
         pc = pc + 1
      end

      self:process()
      log.trace(pc-1, "poses loaded in", sys.toc())
   end
end

function Poses:loaddata(data,names)
   sys.tic()
   self.nposes = data:size(1)
   self.data   = data
   self.names  = names
   print(string.formal("Loading: %s",self.names))
   self:process()
   print(string.format("Loaded %d poses in %2.2fs", self.nposes, sys.toc()))
end

function Poses:process()
   -- quaternion rotation from local xyz to global xyz
   self.quat          = self.data:narrow(2,1,4)

   -- position of origin in global xyz coordinates
   self.xyz           = self.data:narrow(2,5,3)

   -- image center in u and v coordinates
   self.center_u            = self.data:select(2,8)

   -- image center in u and v coordinates
   self.center_v            = self.data:select(2,9)

   -- degree per pixel in x 
   self.degree_per_px_x = self.data:select(2,10)

   -- degree per pixel in y
   self.degree_per_px_y = self.data:select(2,11)

   -- reverse rotation from global to local
   self.quat_r = self.data:narrow(2,12,4)
   self.quat_r:copy(self.quat)
   self.quat_r:narrow(2,1,3):mul(-1)

   -- center x in pixel offset
   self.center_x  = self.data:select(2,16)

   -- center y in pixel offset
   self.center_y  = self.data:select(2,17)

   -- image width in pixels
   self.image_w      = self.data:select(2,18)

   -- image height in pixels
   self.image_h      = self.data:select(2,19)

   -- create individual pose objects
   for i = 1,self.nposes do
      self[i] = Pose.new(self,i)
   end
end

function Poses:save_wireframes_red_alpha(obj,outdir,ext)
   if not outdir then
      outdir = ""
   end
   if not ext then 
      ext = "_wireframe.png"
   end

   for pi = 1,self.nposes do
      local pose = self[pi]
      local wimage = pose:draw_wireframe(obj)
      image.display(wimage)
      -- save
      local wimagename = outdir .. pose.name:gsub(".jpg",ext)
      log.trace("Saving:", wimagename)
      image.save(wimagename,wimage)
   end
end

function Poses:save_wireframes_image_blacklines(obj,outdir,ext)
   if not outdir then
      outdir = ""
   end
   if not ext then 
      ext = "_wireframe_RGB.png"
   end
   for pi = 1,self.nposes do 
      local pose = self[pi]
      local wimage = pose:draw_wireframe(obj)
      -- invert 0 and 1 in red channel
      wimage = wimage[1]:mul(-1):add(1)
      local cimage = pose.image:clone()
      cimage[1]:cmul(wimage)
      cimage[2]:cmul(wimage)
      cimage[3]:cmul(wimage)
      image.display(cimage) 
      local wimagename = outdir .. pose.name:gsub(".jpg",ext)
      if pose.name:gmatch("png") then
         wimagename = outdir .. pose.name:gsub(".png",ext)
      end
      log.trace("Saving:", wimagename)
      image.save(wimagename,cimage)
   end
end


return Poses
