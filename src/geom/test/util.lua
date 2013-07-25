Class()
local geom      = geom.util

data = require "../geom/test/data"

function compute_normals()
   print("Testing compute normals")
   local e          = 0
   local cnt        = 0
   local maxerr     = 0
   local normals    = data.face_plane:narrow(2,1,3)
   local face_verts = data.face_verts
   log.tic()
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
                       e,cnt, maxerr,log.toc()))
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
   angles = angles:t()   
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
   unit_check = unit_check:t()
   local d = torch.abs(unit - unit_check)
   local error = d:gt(1e-15):sum()

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

   unit_check = unit_cartesian:t()
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
   
   angles_check = angles_check:t()

   local d = torch.abs(angles - angles_check)
   local error = d:gt(1e-15):sum()

   print(string.format(" - Found %d/%d errors", error,angles:size(1))) 
end

function sphere_to_unit_and_back()
   print("Testing spherical_angles_to_unit_cartesian and back")
   local npts = 1000
   local uc = torch.randn(3,npts)

   geom.normalize(uc,1)

   local a = geom.unit_cartesian_to_spherical_angles(uc)

   local ucp = geom.spherical_angles_to_unit_cartesian(a)
   local d = torch.abs(ucp - uc)
   local error = d:gt(1e-14):sum()

   print(string.format(" - Found %d/%d errors (max %e)", error, npts,d:max())) 
end

function all()
   compute_normals()
   spherical_angles_to_unit_cartesian()
   -- unit_cartesian_to_spherical_angles()
   sphere_to_unit_and_back()
end

all()
