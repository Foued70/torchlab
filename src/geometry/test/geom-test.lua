local geom      = geometry.util
local test = {}
test.data = require "geometry.test.data.geom-data"

function test.compute_normals()
   print("Testing compute normals")
   local e          = 0
   local cnt        = 0
   local maxerr     = 0
   local normals    = test.data.face_plane:narrow(2,1,3)
   local face_verts = test.data.face_verts
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

function test.all()
   test.compute_normals()
end

return test