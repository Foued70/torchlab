geom = require 'util.geom'

npts = 1000
uc = torch.randn(npts,3)

for i = 1,npts do uc[i]:mul(1/uc[i]:norm()) end

a = geom.unit_cartesian_to_spherical_coords(uc)

ucp = geom.spherical_coords_to_unit_cartesian(a)

for i = 1,npts do 
   d = ucp[i]:dist(uc[i])
   -- trig functions have error of 2 sig bits
   if d > 1e-14 then
      print("["..i.."] "..d)
      print(uc[i])
      print(a[i])
      print(ucp[i])
   end
end
