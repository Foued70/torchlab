local util = require 'util'
local intersect = util.intersect
local geom = util.geom

local test = {}

-- also tests normals and major dimensions
function test.point_in_polygon()
   print("Testing point in polygon")
   local cerr  = 0
   local vperr = 0
   local vmerr = 0
   local verr  = 0
   -- all positive
   local npts  = 1000
   local vs    = torch.randn(npts+3,3)
   local vsmin = vs:min(1)
   vsmin       = vsmin:squeeze()
   local vsmax = vs:max(1)
   vsmax       = vsmax:squeeze()

   for i = 1,npts do 
      local face_verts = vs:narrow(1,i,3)
      local pt         = face_verts:mean(1):squeeze()
      local normal     = geom.compute_normal(face_verts)
      local _,ds = torch.sort(torch.abs(normal))
      ds = ds:narrow(1,1,2)
      if not intersect.point_in_polygon(pt,face_verts,ds) then
         cerr = cerr + 1
      end
      for vi = 1,3 do 
         local vp = face_verts[vi]  
         vp = vp + (pt - vp)*1e-6
         if not intersect.point_in_polygon(vp,face_verts,ds) then 
            vperr = vperr + 1
         end
      end
      for vi = 1,3 do 
         local vm = face_verts[vi]  
         vm = vm + (vm - pt)*1e-6
         if intersect.point_in_polygon(vm,face_verts,ds) then 
            vmerr = vmerr + 1
         end
      end
      for vi = 1,3 do 
         local v = face_verts[vi]
         -- need to pass center point if we expect the points on the
         -- edge to be included in polygon.
         if not intersect.point_in_polygon(v,face_verts,ds,pt) then 
            verr = verr + 1
         end
      end
   end
   printf("-- c:%d/%d vin: %d vout: %d v: %d/%d Errors", 
          cerr, npts, vperr, vmerr, verr, npts*3)
end

function test.ray_boundary_intersect()
   print("Testing ray boundary intersection")
   local o     = torch.Tensor({0,0,0})
   local d     = torch.Tensor({1,1,1})
   local r     = Ray(o,d)
   local rmint = 0
   local rmaxt = math.huge
   local err   = 0
   local tot   = 0
   local root3 = math.sqrt(3)
   function do_tests(r,rdim)
      for dim = 1,3 do 
         local cval = 1
         local bmax = true
         if (rdim == dim) then 
            -- opposite ray direction
            cval = -1 
            bmax = false
         end
         -- fully in: min = -1 (opp. max = 1)
         local tv,mint,maxt = intersect.ray_boundary(r,rmint,rmaxt,dim,-cval,not bmax)
         if (not tv) or (mint ~= 0) or (maxt ~= math.huge) then
               err = err + 1
         end
         -- fully out: max = -1 (opp. min = 1)
         tv = intersect.ray_boundary(r,rmint,rmaxt,dim,-cval,bmax)
         if tv then 
            err = err + 1
         end
         -- crop mint: min = 1 (opp. max = -1)
         tv,mint,maxt = intersect.ray_boundary(r,rmint,rmaxt,dim,cval,not bmax)
         if (not tv) or (math.abs(mint - root3) > 1e-8) or (maxt ~= math.huge) then
            err = err + 1
         end
         -- crop maxt: max = 1 (opp. min = -1)
         tv,mint,maxt = intersect.ray_boundary(r,rmint,rmaxt,dim,cval,bmax)
         if (not tv) or (mint ~= 0) or (math.abs(maxt - root3) > 1e-8) then
            err = err + 1
         end 
         tot = tot + 4
      end
   end
   do_tests(r)

   for rdim = 1,3 do 
      dp = d:clone()
      dp[rdim] = -1
      r = Ray(o,dp)
      do_tests(r,rdim)
   end
   printf(" - Errors: %d/%d", err,tot)
end

-- lots of edge cases when rays intersect corners and edges of the bbox.
function test.ray_bbox_intersect()
   local bbox = torch.Tensor({-1,-1,-1,1,1,1})

   local dirs_axes = torch.Tensor({{ 1, 0, 0},
                                   { 0, 1, 0},
                                   { 0, 0, 1},
                                   {-1, 0, 0},
                                   { 0,-1, 0},
                                   { 0, 0,-1}})
   
   local dirs_planes = torch.Tensor({{ 1, 1, 0},
                                     { 1, 0, 1},
                                     { 0, 1, 1},
                                     {-1,-1, 0},
                                     {-1, 0,-1},
                                     { 0,-1,-1},
                                     { 1,-1, 0},
                                     { 1, 0,-1},
                                     { 0, 1,-1},
                                     {-1, 1, 0},
                                     {-1, 0, 1},
                                     { 0,-1, 1}})
   
   local dirs_45deg = torch.Tensor({{ 1, 1, 1},
                                    {-1, 1, 1},
                                    { 1,-1, 1},
                                    { 1, 1,-1},
                                    {-1,-1, 1},
                                    { 1,-1,-1},
                                    {-1, 1,-1},
                                    {-1,-1,-1}})

   local origins = torch.Tensor({{ 1, 0, 0},
                                 { 0, 1, 0},
                                 { 0, 0, 1}})
    
   function test_dirs(dirs,o,verbose)
      local tmin_max = torch.Tensor(dirs:size(1),2) 
      for di = 1,dirs:size(1) do 
         local d = dirs[di]
         local r = Ray(o,d)
         local tval, tmin,tmax = intersect.ray_bbox(r,bbox)
         local rn = r(tmin)
         local rx = r(tmax)
         tmin_max[di][1] = tmin
         tmin_max[di][2] = tmax
         if verbose then 
            printf(" -  dir: (%2.2f,%2.2f,%2.2f)",d[1],d[2],d[3])
            printf(" - tmin: %2.2f ->  (%2.2f,%2.2f,%2.2f)",tmin,
                   rn[1],rn[2],rn[3])
            printf(" - tmax: %2.2f ->  (%2.2f,%2.2f,%2.2f)",tmax,
                   rx[1],rx[2],rx[3])
         end
      end
      return tmin_max
   end

   local errs = 0
   print("** INSIDE at origin test (all hit) ***")
   local o = torch.Tensor({0,0,0})
   print("Test origin at (0,0,0)")
   print("+ Test axes")
   local tmx = test_dirs(dirs_axes,o)
   printf("  min errors: %d/%d",
          torch.sum(torch.ne(tmx:select(2,1),0)),
          dirs_axes:size(1))
   printf("  max errors: %d/%d",
          torch.sum(torch.ne(tmx:select(2,2),1)),
          dirs_axes:size(1))

   print("+ Test planes (hit edges)")
   tmx = test_dirs(dirs_planes,o)
   printf("  min errors: %d/%d",
          torch.sum(torch.ne(tmx:select(2,1),0)),
          dirs_planes:size(1))
   printf("  max errors: %d/%d",
          torch.sum(torch.gt(torch.abs(tmx:select(2,2):add(-math.sqrt(2))),1e-8)),
          dirs_planes:size(1))

   print("+ Test degrees (hit corners)")
   tmx = test_dirs(dirs_45deg,o)
   printf("  min errors: %d/%d",
          torch.sum(torch.ne(tmx:select(2,1),0)),
          dirs_45deg:size(1))
   printf("  max errors: %d/%d",
          torch.sum(torch.gt(torch.abs(tmx:select(2,2):add(-math.sqrt(3))),1e-8)),
          dirs_planes:size(1))
   

   print("** INSIDE near corners test (all should hit) ***")
   for oi = 1,dirs_45deg:size(1) do 
      o = dirs_45deg[oi] * 0.9   
      printf("Test origin at (%f,%f,%f)",o[1],o[2],o[3])
      print("+ Test axes")
      tmx = test_dirs(dirs_axes,o)
      printf("  min errors: %d/%d",
             torch.sum(torch.ne(tmx:select(2,1),0)),
             dirs_axes:size(1))
      printf("  max errors: %d/%d",
             dirs_axes:size(1) - torch.sum(torch.lt(tmx:select(2,2),math.huge)),
             dirs_axes:size(1))
      print("+ Test planes")
      tmx = test_dirs(dirs_planes,o)
      printf("  min errors: %d/%d",
             torch.sum(torch.ne(tmx:select(2,1),0)),
             dirs_planes:size(1))
      printf("  max errors: %d/%d",
             dirs_planes:size(1) - torch.sum(torch.lt(tmx:select(2,2),math.huge)),
             dirs_planes:size(1))
      print("+ Test degrees")
      tmx = test_dirs(dirs_45deg,o)
      printf("  min errors: %d/%d",
             torch.sum(torch.ne(tmx:select(2,1),0)),
             dirs_45deg:size(1))
      printf("  max errors: %d/%d",
             dirs_45deg:size(1) - torch.sum(torch.lt(tmx:select(2,2),math.huge)),
             dirs_45deg:size(1))
   end

--    print("** OUTSIDE near corners test (all miss) ***")
--    for oi = 1,dirs_45deg:size(1) do 
--       o = dirs_45deg[oi] * 1.1   
--       printf("Test origin at (%f,%f,%f)",o[1],o[2],o[3])
--       test_axes(o)
--    end

--    print("** annoying Boundary test (origin on corners some hit) ***")

--    for oi = 1,dirs_45deg:size(1) do 
--       o = dirs_45deg[oi]   
--       printf("Test origin at (%f,%f,%f)",o[1],o[2],o[3])
--       print("+ Test axes")
--       tmx = test_dirs(dirs_axes,o,true)
--       print("+ Test planes")
--       tmx = test_dirs(dirs_planes,o,true)
--       print("+ Test degrees")
--       tmx = test_dirs(dirs_45deg,o,true)
      
--    end

end

function test.all()
   test.point_in_polygon()
   test.ray_boundary_intersect()
   test.ray_bbox_intersect()
end

return test
