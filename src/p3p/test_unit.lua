
npts = 1000
uc = torch.randn(npts,3)

for i = 1,npts do uc[i]:mul(1/uc[i]:norm()) end

a = geom.util.unit_cartesian_to_spherical_angles(uc)

ucp = geom.util.spherical_angles_to_unit_cartesian(a)

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
