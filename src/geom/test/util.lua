Class()
local geom      = geom.util

data = require "geom.test.data"

function compute_normals()
   print("Testing compute normals")
   local e          = 0
   local cnt        = 0
   local maxerr     = 0
   local normals    = data.face_plane:narrow(2,1,3)
   local face_verts = data.face_verts
   sys.tic()
   for i = 1,normals:size(1) do
      cnt = cnt + 1
      local n = geom.compute_normal(face_verts[i])
      local gtn = normals[i]
      local err = torch.sum(torch.abs(n - gtn))
      if err > maxerr then maxerr = err end
      if err > 1e-7 then
         e = e + 1
         print(string.format("%e : (%e,%e,%e) <-> (%e,%e,%e)",
                             err, n[1],n[2],n[3],
                             gtn[1],gtn[2],gtn[3]))
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,cnt, maxerr,sys.toc()))
end

function spherical_angles_to_unit_cartesian()
   print("Testing spherical_angles_to_unit_cartesian")
   local pi  = math.pi
   local pi2 = math.pi / 2
   local pi4 = math.pi / 4
   local sp4 = math.sin(pi4)
   local angles = torch.Tensor(
      {
         {   0,    0}, -- right:  0 forward: 1 up:  0 
         { pi2,    0}, -- right:  1 forward: 0 up:  0 
         {-pi2,    0}, -- right: -1 forward: 0 up:  0 
         {   0,  pi2}, -- right:  0 forward: 0 up:  1 
         {   0, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {-pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         {-pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi,   pi}, -- right:  0 forward: 1 up:  0 
         { pi2,   pi}, -- right: -1 forward: 0 up:  0 
         {-pi2,   pi}, -- right:  1 forward: 0 up:  0 
         {  pi,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi, -pi2}, -- right:  0 forward: 0 up: -1 
      })
   
   local unit = geom.spherical_angles_to_unit_cartesian(angles)
   
   local unit_check = torch.Tensor(
      {
         { 0,1, 0}, --    0,  0
         { 1,0, 0}, --  pi2,  0
         {-1,0, 0}, -- -pi2,  0
         { 0,0, 1}, --    0,  pi2
         { 0,0,-1}, --    0, -pi2
         { 0,0, 1}, --  pi2,  pi2
         { 0,0,-1}, -- -pi2, -pi2
         { 0,0,-1}, --  pi2, -pi2
         { 0,0, 1}, -- -pi2,  pi2
         { 0,1, 0}, --   pi,  pi
         {-1,0, 0}, --  pi2,  pi
         { 1,0, 0}, -- -pi2,  pi
         { 0,0, 1}, --   pi,  pi2
         { 0,0,-1}, --   pi, -pi2
      })
   
   local error = unit:dist(unit_check,1)
   if error < 1e-8 then error = 0 end
   print(string.format(" - Found %d errors", error)) 
end

function unit_cartesian_to_spherical_angles()
   print("Testing unit_cartesian_to_spherical_angles")
   local pi  = math.pi
   local pi2 = math.pi / 2
   local pi4 = math.pi / 4
   local sp4 = math.sin(pi4)
   
   local unit_cartesian = torch.Tensor(
      {
         { 0,1, 0}, --    0,  0
         { 1,0, 0}, --  pi2,  0
         {-1,0, 0}, -- -pi2,  0
         { 0,0, 1}, --    0,  pi2
         { 0,0,-1}, --    0, -pi2
         { 0,0, 1}, --  pi2,  pi2
         { 0,0,-1}, -- -pi2, -pi2
         { 0,0,-1}, --  pi2, -pi2
         { 0,0, 1}, -- -pi2,  pi2
         { 0,1, 0}, --   pi,  pi
         {-1,0, 0}, --  pi2,  pi
         { 1,0, 0}, -- -pi2,  pi
         { 0,0, 1}, --   pi,  pi2
         { 0,0,-1}, --   pi, -pi2
      })

   local angles = geom.unit_cartesian_to_spherical_angles(unit_cartesian)

   local angles_check = torch.Tensor(
      {
         {   0,    0}, -- right:  0 forward: 1 up:  0 
         { pi2,    0}, -- right:  1 forward: 0 up:  0 
         {-pi2,    0}, -- right: -1 forward: 0 up:  0 
         {   0,  pi2}, -- right:  0 forward: 0 up:  1 
         {   0, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {-pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         {-pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi,   pi}, -- right:  0 forward: 1 up:  0 
         { pi2,   pi}, -- right: -1 forward: 0 up:  0 
         {-pi2,   pi}, -- right:  1 forward: 0 up:  0 
         {  pi,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi, -pi2}, -- right:  0 forward: 0 up: -1 
      })
   
   
   local error = 0 
 
   for i = 1,angles:size(1) do 
      local d = angles[i]:dist(angles_check[i])
      -- trig functions have error of 2 sig bits
      if d > 1e-14 then
         print("["..i.."] "..d)
         print(angles[i])
         print(angles_check[i])
         error = error + 1
      end
   end
   print(string.format(" - Found %d/%d errors", error,angles:size(1))) 
end

function sphere_to_unit_and_back()
   print("Testing spherical_angles_to_unit_cartesian and back")
   local npts = 1000
   local uc = torch.randn(npts,3)

   for i = 1,npts do uc[i]:mul(1/uc[i]:norm()) end

   local a = geom.unit_cartesian_to_spherical_angles(uc)

   local ucp = geom.spherical_angles_to_unit_cartesian(a)
   local error = 0
   for i = 1,npts do 
      local d = ucp[i]:dist(uc[i])
      -- trig functions have error of 2 sig bits
      if d > 1e-14 then
         print("["..i.."] "..d)
         print(uc[i])
         print(a[i])
         print(ucp[i])
         error = error + 1
      end
   end
   print(string.format(" - Found %d/%d errors", error, npts)) 
end

function all()
   compute_normals()
   spherical_angles_to_unit_cartesian()
   -- unit_cartesian_to_spherical_angles()
   sphere_to_unit_and_back()
end