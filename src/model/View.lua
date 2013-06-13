local View = Class()

local normalize = geom.util.normalize

function View:__init(position, orientation, hfov, vfov, clip_distance)

   local n_planes = 5
   if clip_distance then 
      n_planes = 6
   end

   -- v,h +,+|+,-|-,-|-,+|
   local frustrum_angles = torch.Tensor({{vfov,vfov,-vfov,-vfov,0},{hfov,-hfov,-hfov,hfov,0}})
   frustrum_angles:mul(0.5)
   local frustrum_quat = geom.quaternion.from_euler_angle(frustrum_angles)
   -- move frustrum quat to global orientation
   frustrum_quat = geom.quaternion.product(orientation,frustrum_quat)
   local frustrum_vect = geom.quaternion.rotate(frustrum_quat,geom.quaternion.y)

   local frustrum_plane = torch.zeros(n_planes,4)
   local prev = frustrum_vect[4]
   for v = 1,4 do
      local cur = frustrum_vect[v]
      frustrum_plane[{v,{1,3}}] = normalize(torch.cross(prev,cur))
      prev = cur
   end
   frustrum_plane[{5,{1,3}}] = frustrum_vect[5]

   if clip_distance then 
      frustrum_plane[{6,{1,3}}] = frustrum_vect[5]
      -- far plane points in the opposite direction
      frustrum_plane[{6,{1,3}}]:mul(-1)
   end

   -- compute d for each plane normal vector
   frustrum_plane[{{},4}] =
      torch.cmul(frustrum_plane[{{},{1,3}}],position:reshape(1,3):expand(n_planes,3)):sum(2):mul(-1)

   if clip_distance then 
      local far_point = position + (frustrum_vect[5] * clip_distance)
      frustrum_plane[6][4] = torch.dot(far_point,frustrum_plane[{6,{1,3}}])
   end

   self.position = position
   self.rotation = orientation
   self.hfov     = hfov
   self.vfov     = vfov
   self.frustrum = frustrum_plane
   self.n_planes = n_planes
end