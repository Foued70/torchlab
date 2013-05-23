Class()

local Ray =  geom.Ray

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


-- For the BIH intersection you only check one bound for each child,
-- not even the two bounds of a slab and not the 3 slabs of a bbox.
-- It is important to keep track of the direction of the ray and
-- whether the bound is a max or a min.
function ray_boundary(ray,ray_mint,ray_maxt,c_dim,c_val,c_ismax)
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

function ray_interval(ray,dim,imin,imax)
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

function ray_polygon(ray,obj,fid,debug)
   local orig = ray.origin
   local  dir = ray.dir
   local norm = obj.face_normals[fid]
   local    d = obj.face_center_dists[fid]
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
   local intersection = ray:endpoint(t)
   -- precompute
   local _,ds   = torch.sort(torch.abs(obj.face_normals[fid]))
   local nverts = obj.n_verts_per_face[fid]
   local verts  = obj.face_verts[fid]:narrow(1,1,nverts)
   local center = obj.face_centers[fid]
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
function ray_bbox(ray,bbox)
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

-- FIXME make tests as this function seems to invert results for
-- simple axis-aligned plane intersections

function ray_plane(...)
   local pt,dir,plane_norm,plane_d,debug
   local args  = {...}
   local nargs = #args
   if nargs == 5 then
      debug    = args[5]
   end
   if nargs < 4 then
      print(dok.usage('ray_plane_intersection',
                      'does ray intersect the plane',
                      '> returns: intersection,distance or nil,errno',
                      {type='torch.Tensor', help='point (start of ray)'},
                      {type='torch.Tensor', help='direction (of ray)', req=true},
                      {type='torch.Tensor', help='normal (of plane)', req=true},
                      {type='torch.Tensor', help='offset (of plane)', req=true},
                      {type='torch.Tensor', help='debug', default=False}))

      dok.error('incorrect arguements', 'ray_plane_intersection')
   else
      pt         = args[1]
      dir        = args[2]
      plane_norm = args[3]
      plane_d    = args[4]
   end
   local a = torch.dot(plane_norm,dir)
   if torch.abs(a) < 1e-8 then
      if debug then print(" - angle parallel") end
      return nil,1
   end
   local t = -(torch.dot(plane_norm,pt) + plane_d)/a
   if debug then print(string.format(" - t: %f", t)) end
   if t < 0 then
      if debug then print(" - plane behind") end
      return nil,2
   end
   local i = pt + dir * t
   if debug then print(string.format(" - int: [%f, %f, %f]",
                                     i[1],i[2],i[3])) end
   return i,t
end


function ray_face(...)
   local pt,dir,plane_norm,plane_d,face_verts,debug
   local args  = {...}
   local nargs = #args
   if nargs == 6 then
      debug    = args[6]
   end
   if nargs < 4 then
      print(dok.usage('ray_face_intersection',
                      'does ray intersect the face',
                      '> returns: intersection point,distance, or nil,errno',
                      {type='torch.Tensor', help='point (start of ray)'},
                      {type='torch.Tensor', help='direction (of ray)', req=true},
                      {type='torch.Tensor', help='normal (of plane)', req=true},
                      {type='torch.Tensor', help='offset (of plane)', req=true},
                      {type='torch.Tensor', help='face vertices', req=true},
                      {type='torch.Tensor', help='debug', default=False}))

      dok.error('incorrect arguements', 'ray_plane_intersection')
   else
      pt         = args[1]
      dir        = args[2]
      plane_norm = args[3]
      plane_d    = args[4]
      face_verts = args[5]
   end
   -- First find planar intersection
   local intersection,t =
      ray_plane(pt,dir,plane_norm,plane_d,debug)
   if not intersection then
      return nil,t
   end
   -- pick two most planar dimensions of the face throw away
   -- coordinate with greatest magnitude (if normal is mostly z
   -- then we want x and y)
   local nverts = face_verts:size(1)
   local _,ds = torch.sort(torch.abs(plane_norm))
   local ri = torch.Tensor(2)
   local verts = torch.Tensor(nverts,2)
   ri[1] = intersection[ds[1]]
   ri[2] = intersection[ds[2]]
   for i = 1,nverts do
      verts[i][1] = face_verts[i][ds[1]] - ri[1]
      verts[i][2] = face_verts[i][ds[2]] - ri[2]
   end
   -- count crossings along 'y' axis : b in slope intercept line equation
   local pvert = verts[nverts]
   local count = 0
   for vi = 1,nverts do
      local cvert = verts[vi]
      --  compute y axis crossing (b = y - mx)
      local run  =  cvert[1] - pvert[1]
      local b    = -math.huge
      local cpos = 1
      local ppos = 1
      if math.abs(run) < 1e-8 then
         if (math.abs(cvert[1]) < 1e-8) then
            count = count + 1
         end
      else
         b = cvert[2] - ((cvert[2] - pvert[2])/run) * cvert[1]
         if (cvert[1] < 0) then cpos = -1 end
         if (pvert[1] < 0) then ppos = -1 end
         if (b >= 0) and ((cpos + ppos) == 0) then
            count = count + 1
         end
      end
      pvert = cvert
   end
   if ((count > 0) and (count % 2)) then
      return intersection,t
   else
      return nil,3
   end
end
