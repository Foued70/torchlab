local util = require 'util'
local geom = util.geom
local Ray  = util.Ray


-- pt (point) is the intersection between the ray and the plane of the
-- polygon verts are the vertices of the polygon dims are the
-- precomputed dominant dimensions in which we flatten the polygon to
-- compute
function point_in_polygon(pt,verts,dims,center)
   -- fudge to move points slightly towards the center
   local eps = 1e-2 -- 1cm 
   if center then
      pt = pt + (center - pt)*eps
   end
   -- dims 1 == X axis
   -- dims 2 == Y axis
   local x         = dims[1]
   local y         = dims[2]
   local nverts    = verts:size(1)
   local inside    = false
   local p1        = verts[nverts] 
   local yflag1    = (p1[y] >= pt[y])
   for vi = 1,nverts do
      local p2     = verts[vi]
      local yflag2 = (p2[y] >= pt[y])
      -- do we have a potential crossing ? p1 and p2 on either side of pt
      if (yflag1 ~= yflag2) then
         -- no division test of positive intersection with xaxis
         if (yflag2 == 
             ((p2[y] - pt[y])*(p1[x] - p2[x]) >= (p2[x] - pt[x])*(p1[y] - p2[y]))) then
            inside = not inside
         end
      end
      yflag1 = yflag2
      p1 = p2
   end
   return inside
end

-- also tests normals and major dimensions
function test_point_in_polygon()
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
      if not point_in_polygon(pt,face_verts,ds) then
         cerr = cerr + 1
      end
      for vi = 1,3 do 
         local vp = face_verts[vi]  
         vp = vp + (pt - vp)*1e-6
         if not point_in_polygon(vp,face_verts,ds) then 
            vperr = vperr + 1
         end
      end
      for vi = 1,3 do 
         local vm = face_verts[vi]  
         vm = vm + (vm - pt)*1e-6
         if point_in_polygon(vm,face_verts,ds) then 
            vmerr = vmerr + 1
         end
      end
      for vi = 1,3 do 
         local v = face_verts[vi]
         -- need to pass center point if we expect the points on the
         -- edge to be included in polygon.
         if not point_in_polygon(v,face_verts,ds,pt) then 
            verr = verr + 1
         end
      end
   end
   printf("-- c:%d/%d vin: %d vout: %d v: %d/%d Errors", 
          cerr, npts, vperr, vmerr, verr, npts*3)
end

-- For the BIH intersection you only check one bound for each child,
-- not even the two bounds of a slab and not the 3 slabs of a bbox.
-- It is important to keep track of the direction of the ray and
-- whether the bound is a max or a min.
function ray_boundary_intersect(ray,ray_mint,ray_maxt,c_dim,c_val,c_ismax)
   local t1 = ((c_val - ray.origin[c_dim]) * ray.invdir[c_dim])
   local ray_ispos = (ray.sign[c_dim] == 1)
   local dir = (c_ismax and  ray_ispos) or (not c_ismax and not ray_ispos)
   if (dir) then
      -- pos dir means ray will get maxt clipped (r: -> max: <-)
      if (t1 < ray_mint) then
         -- ray does not cross
         return false
      elseif (t1 >= ray_maxt) then
         -- ray segment fully inside the boundary 
         return true, ray_mint, ray_maxt
      else
         -- ray is clipped
         return true, ray_mint, t1
      end 
   else 
      -- neg dir means ray will get mint clipped (r: -> min: ->)
      if (t1 <= ray_mint) then 
         -- ray segment fully inside the boundary 
         return true, ray_mint, ray_maxt
      elseif (t1 <= ray_maxt) then
         -- intersect and clip
         return true, t1, ray_maxt
      else
         return false
      end
   end
   
end

function test_ray_boundary_intersect()
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
         local tv,mint,maxt = ray_boundary_intersect(r,rmint,rmaxt,dim,-cval,not bmax)
         if (not tv) or (mint ~= 0) or (maxt ~= math.huge) then
               err = err + 1
         end
         -- fully out: max = -1 (opp. min = 1)
         tv = ray_boundary_intersect(r,rmint,rmaxt,dim,-cval,bmax)
         if tv then 
            err = err + 1
         end
         -- crop mint: min = 1 (opp. max = -1)
         tv,mint,maxt = ray_boundary_intersect(r,rmint,rmaxt,dim,cval,not bmax)
         if (not tv) or (math.abs(mint - root3) > 1e-8) or (maxt ~= math.huge) then
            err = err + 1
         end
         -- crop maxt: max = 1 (opp. min = -1)
         tv,mint,maxt = ray_boundary_intersect(r,rmint,rmaxt,dim,cval,bmax)
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

function ray_interval_intersect(ray,dim,imin,imax)
   local tnear = (imin - ray.origin[dim]):cmul(ray.invdir[dim])
   local tfar  = (imax - ray.origin[dim]):cmul(ray.invdir[dim])

   local t0 = ray.mint
   local t1 = ray.maxt

   local swapv = 0
   if (tnear > tfar) then
         swapv = tnear
         tnear = tfar
         tfar  = swapv
   end
   if (tnear > t0) then t0 = tnear end
   if (tfar  < t1) then t1 = tfar end
   if (t0 > t1) then return false, ray.mint, ray.maxt end
   return true, t0,t1
end

function ray_polygon_intersection(ray,obj,fid,debug)
   local orig = ray.origin
   local  dir = ray.dir
   local norm = obj.normals[fid]
   local    d = obj.d[fid]
   local    a = norm:dot(dir)
   if torch.abs(a) < 1e-8 then 
      return nil
   end

   local    t = -(norm:dot(orig) + d)/a
   if t < 0 then 
      return nil
   end
   if debug then
      printf("  - %d %f", fid, t)
      -- printf("  - bbox\n%s", obj.face_bboxes[fid]:resize(2,3))
   end
   local intersection = ray(t)
   -- precompute
   local _,ds   = torch.sort(torch.abs(obj.normals[fid]))
   local nverts = obj.nverts_per_face[fid]
   local verts  = obj.face_verts[fid]:narrow(1,1,nverts)
   local center = obj.centers[fid]
   local found  = point_in_polygon(intersection,verts,ds,center)
   if debug then
      printf("  - %s", found)
   end
   if found then 
      return t
   else
      return nil
   end
end

-- intersection with Axis-aligned bbox (see pbrt book p.194)
function ray_bbox_intersect(ray,bbox)
   local bbmin = bbox:narrow(1,1,3)
   local bbmax = bbox:narrow(1,4,3)
   local tnear = (bbmin - ray.origin):cmul(ray.invdir)
   local tfar  = (bbmax - ray.origin):cmul(ray.invdir)

   local t0 = ray.mint
   local t1 = ray.maxt

   local swapv = 0
   for i = 1,3 do 
      if (tnear[i] > tfar[i]) then
         swapv    = tnear[i]
         tnear[i] = tfar[i]
         tfar[i]  = swapv
      end
      if (tnear[i] > t0) then t0 = tnear[i] end
      if (tfar[i]  < t1) then t1 = tfar[i] end
      if (t0 > t1) then return false, ray.mint, ray.maxt end
   end
   return true, t0,t1
end

-- lots of edge cases when rays intersect corners and edges of the bbox.
function test_ray_bbox_intersect()
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
         local tval, tmin,tmax = ray_bbox_intersect(r,bbox)
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

function intersection_run_all_tests()
   test_point_in_polygon()
   test_ray_boundary_intersect()
   test_ray_bbox_intersect()
   -- test_ray_plane_intersection()
   -- test_ray_face_intersection()
end
-- run_all_tests()
