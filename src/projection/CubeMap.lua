local CubeMap = Class()

local pi  = math.pi
local pi2 = pi/2

-- wrapper of a bunch of Gnomonic Projections to create a CubeMap

function CubeMap:__init(projection_from, out_size)

   self.projection_from = projection_from

   self.width = out_size
   self.height = out_size
   self.hfov = pi2
   self.vfov = pi2
   self.pixel_center_x = out_size/2
   self.pixel_center_y = out_size/2

   -- make a skybox
   self.centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}
   -- this naming comes from unity and is from the outside looking in
   self.names   = {"front", "left", "back", "right", "down", "up"}
   
   self.remappers = {}
   for _,off in ipairs(self.centers) do 
      
      local proj_to = 
         projection.GnomonicProjection.new(self.width,self.height,
                                           self.hfov, self.vfov,
                                           self.pixel_center_x, self.pixel_center_y,
                                           off[1],off[2])
      
      table.insert(self.remappers, projection.Remap.new(self.projection_from,proj_to))
   end

end


function CubeMap:remap(img)
   local faces = {}
   local names = self.names
   for i,r in ipairs(self.remappers) do 
      faces[names[i]] = r:remap(img)
   end
   return faces
end
