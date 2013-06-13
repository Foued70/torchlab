local View = Class()


function View:__init(position, orientation, hfov, vfov)
      
      printf("optical axis quat: %f, %f, %f, %f ",
             orientation[1],orientation[2],orientation[3],orientation[4])
      
      -- v,h +,+|+,-|-,-|-,+
      local frustrum_angles = torch.Tensor({{vfov,vfov,-vfov,-vfov},{hfov,-hfov,-hfov,hfov}})
      frustrum_angles:mul(0.5)
      local frustrum_quat = geom.quaternion.from_euler_angle(frustrum_angles)
      -- move frustrum quat to global orientation
      frustrum_quat = geom.quaternion.product(orientation,frustrum_quat)
      
      local frustrum_vect = geom.quaternion.rotate(frustrum_quat,geom.quaternion.y)
      
      print("fq:"); print(frustrum_quat)
      print("fv:"); print(frustrum_vect)
      
      local frustrum_plane = torch.Tensor(6,4)
      local prev = frustrum_vect[4]
      for v = 1,4 do 
         local cur = frustrum_vect[v]
         frustrum_plane[{v,{1,3}}] = torch.cross(prev,cur)
         prev = cur
      end
      print("fp:"); print(frustrum_plane)
      
      self.position = position
      self.rotation = orientation
      self.hfov     = hfov
      self.vfov     = vfov
      self.frustrum = frustrum_plane
end