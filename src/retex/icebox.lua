-- FIXME make tests as this function seems to invert results for
-- simple axis-aligned plane intersections

function ray_plane_intersection(...)
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
      

function ray_face_intersection(...)
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
      ray_plane_intersection(pt,dir,plane_norm,plane_d,debug)
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

-- Intentionally very slow.  Checks _all_ the faces. Returns closest
-- intersection. Used for debugging the aggregates.
function get_occlusions(ray,obj,debug)
   local mindepth        = math.huge
   local fid             = 0
   local nverts_per_face = obj.nverts_per_face
   local face_verts      = obj.face_verts
   local normals         = obj.normals
   local ds              = obj.d
   -- exhausting loop through all faces
   for fi = 1,obj.nfaces do
      local nverts = nverts_per_face[fi]
      local verts  = face_verts[fi]:narrow(1,1,nverts)      
      local normal = normals[fi]
      local d      = ds[fi]

      local testd = ray_polygon_intersection(ray,obj,fi,debug)
      local bstr  = " "
      if testd and (testd < mindepth) then
         bstr = "*"
         mindepth = testd
         fid = fi
      end
      if debug then 
         if not testd then testd = math.huge end
         printf("%s[%05d] : %f", bstr,fi,testd)
      end
      
   end
   return mindepth,fid
end

function fast_ray_face_intersection(pt,dirs,
                                    norms,d,
                                    face_verts,most_planar_normals,
                                    nverts_per_face,
                                    ...)
   -- optional debugging arguments
   local args = {...}
   local groundt = nil
   local fid = nil
   if (#args > 0) then
      groundt = args[1]
      if (#args > 1) then 
         fid = args[2]
      end
   end

   local debug = false

   local timer = torch.Timer.new()

   -- output depth map (FIXME should pass in if we want to fill and avoid the copy)
   local dmap  = torch.Tensor(dirs:size(1)):fill(0)

   -- nfaces x ndirections
   local distances = fast_ray_plane_intersection(pt,dirs,norms,d)
   local time0 = timer:time()
   printf("Compute distances: %2.4fs",time0.real)

   -- angle between plane_norms and direction
   -- angles < 0 and at inf can be skipped
   local dists,index = torch.sort(distances,1)

   -- dists < 0 are behind camera and are not to be processed to avoid
   -- numerical issues distances less than 1000m are avoided (don't
   -- know of a model with 1km walls)
   local valid = torch.gt(dists,0):cmul(torch.lt(dists,1000))
   local vsum  = valid:sum(1):squeeze()
   local time1 = timer:time()
   printf("Multiply and sort: %2.4fs",time1.real-time0.real)

   local nfaces = face_verts:size(1)
   local temp_verts = torch.Tensor(face_verts:size(2),face_verts:size(3))
   local time2 = timer:time()
   printf("Compute planar norms: %2.4fs",time2.real-time1.real)


   local time_3_1 = timer:time()
   local acc3_1 = 0
   local acc3_2 = 0
   local acc3_3 = 0
   local acc3_4 = 0
   local acc3_5 = 0

   local visits = 0

   -- loop over directions in the distances. 
   -- Each direction has a set of distances to each face
   for i = 1,distances:size(2) do
      local j = 1 
      local found = false
      local visited = 0
      if vsum[i] > 0 then
         time3_1 = timer:time()
         local v   = valid[j][i]
         local dir = dirs[i]
         -- FIXME replace with new get_lt bsearch function 
         while (v == 0) and (j < (nfaces + 1)) do
            j = j + 1
            v = valid[j][i]
         end
         local time3_2 = timer:time()
         acc3_2 = acc3_2 + time3_2.real - time3_1.real

         while (not found) and (v == 1) and (j < (nfaces + 1)) do
            local time3_2_1 = timer:time()
            visited    = visited + 1
            v          = valid[j][i]
            local fidx = index[j][i]
            local    t = dists[j][i]
            
            -- FIXME do a batch of directions at a time
            local intersection = pt + dir * t
            local nverts       = nverts_per_face[fidx]
            
            local ds           = most_planar_normals[fidx]

            local time3_3 = timer:time()
            acc3_3 = acc3_3 + time3_3.real - time3_2_1.real

            -- faster intersection (no copies less substraction)
            local verts = face_verts[fidx]:narrow(1,1,nverts)
            found       = point_in_polygon(intersection,verts,ds)
            local time3_4 = timer:time()
            acc3_4 = acc3_4 + time3_4.real - time3_3.real
            if found then 
               dmap[i] = t
            else
               j = j + 1 
            end
         end -- while
         local time3_5 = timer:time()
         acc3_5 = acc3_5 + time3_5.real - time3_2.real
      end -- if valid
      visits = visits + visited
      if groundt and (torch.abs(dmap[i] - groundt[i]) > 1e-4) then
         --print(valid:select(2,i))
         print(index:select(2,i))
         -- print(dists:select(2,i))
         --print(distances:select(2,i))
         printf("[%d][%d]",i,j)
         local idx = j
         if not found then
            printf("Intersection for direction %d NOT FOUND",i)
         else
            idx = index[j][i] 
         end
         printf("dir: %d face: %d <-> %d %2.4f <-> %2.4f = %2.4f visited: %d", 
                i, idx, fid[idx], 
                dmap[i], groundt[i], dmap[i] - groundt[i], 
                visited)
      end
   end -- for each direction 
   local time4 = timer:time()
   printf(" - scanning              : %2.4fs",acc3_2)
   printf(" - setup                 : %2.4fs",acc3_3)
   printf(" - point in polygon(new) : %2.4fs",acc3_4)
   printf(" - scan + intersect outer: %2.4fs",acc3_5)
   printf("total face intersections : %2.4fs",time4.real-time2.real)
   printf("    ***   TOTAL  ***     : %2.4fs",time4.real)
   printf("visits per ray: %2.2f",visits/distances:size(2))
   return dmap

end


-- takes a bunch of rays (pt, dirs) and a bunch of planes (normals,d)
-- and computes the distance to all planes for each direction using
-- matrix operations.
function fast_ray_plane_intersection(pt,dirs,normals,d)
   -- compute distance along ray (pt, in all directions dir) to all
   -- face planes pow(-1) is robust to 0s. will place inf in cell
   -- distances is nfaces * ndirs 
   -- matrix version of : angles = 1/(normal dot dir)
   local distances = torch.mm(normals,dirs:t()):pow(-1)
   
   -- ts is vector of nfaces
   -- normals dot xyz + plane_d (t's are distance along the ray where
   -- the plane intersection happens

   local ts     = - (torch.mv(normals,pt) + d)

   -- iterate over directions here FIXME move to ffi
   for i = 1,distances:size(2) do
      distances:select(2,i):cmul(ts)
   end

   return distances
end

function fast_get_occlusions(p,pi,obj,scale,ps)
   local timer = torch.Timer.new()

   -- 1) set up local variables
   local dirs = load_dirs(p,pi,scale,ps)
   local time0 = timer:time()

   printf("Compute dirs: %2.4fs",time0.real)

   local pt    = p.xyz[pi] -- select pose xyz

   -- output depth map
   local dmap  = torch.Tensor(dirs:size(1)*ps,dirs:size(2)*ps):fill(0)

   -- FIXME pass obj to face intersection
   local norms           = obj.normals
   local d               = obj.d
   local face_verts      = obj.face_verts
   local nverts_per_face = obj.nverts_per_face

   -- 2) precompute dominant dimensions for all edges of
   -- all faces precompute the differences between consecutive face
   -- verts used for the line eq. when testing for intersection. FIXME
   -- move this to a face object. FIXME move to obj.

   -- precompute the indexes of the dominant dimensions for each face 
   local nfaces = face_verts:size(1)
   local most_planar_normals = torch.IntTensor(nfaces,2)
   for i = 1,nfaces do
      local _,ds = torch.sort(torch.abs(norms[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end

   local orow = 1
   local ocol = 1
   local ray_packet = nil
   for r = 1,dirs:size(1) do 
      for c = 1,dirs:size(2) do 
         printf("[%d][%d] of [%d][%d]",orow,ocol,dmap:size(1),dmap:size(2))
         ray_packet = dmap:narrow(1,orow,ps):narrow(2,ocol,ps)
         ray_packet:copy( 
            fast_ray_face_intersection(pt,dirs[r][c],
                                       norms,d,
                                       face_verts,most_planar_normals,
                                       nverts_per_face))
         ocol = ocol + ps
      end
      orow = orow + ps
      ocol = 1
   end
   
   return dmap
end

function test_ray_plane_intersection(poses,pi,scale)
   print("Testing ray plane intersections")
   if not poses then
      print("Error must pass pose table for testing")
   end
   if not pi then
      pi = 1
   end
   if not scale then
      scale = 8
   end
   local dirs    = grid_contiguous(compute_dirs(poses,pi,scale),32,32)

   dirs = dirs[3][4]

   local obj     = target
   local norms   = obj.normals
   local ds       = obj.d

   log.tic()

   local pt      = poses.xyz[pi] 

   local dots    = torch.mm(norms,dirs:t())
   local invdots = torch.pow(dots,-1)
   local ts      = - ( torch.mv(norms,pt) + ds)
   
   local dists   = torch.Tensor(dots:size()):copy(invdots)

   -- iterate over directions 
   for i = 1,dists:size(2) do
      dists:select(2,i):cmul(ts) 
   end

   -- nfaces x ndirections
   local distances = fast_ray_plane_intersection(pt,dirs,norms,ds)
   local nerr      = 0
   local dperr     = 0
   local idoterr   = 0
   local initerr   = 0
   local finerr    = 0
   
   for fi = 1,norms:size(1) do
      local norm = norms[fi]
      local d = ds[fi]
      for di = 1,dirs:size(1) do
         local errp = false
         local dir = dirs[di]
         local errstr = string.format(
            "[%d][%d] dir: (%2.2f,%2.2f,%2.2f) norm: (%2.2f,%2.2f,%2.2f)",
            fi,di,
            dir[1], dir[2],dir[3], 
            norm[1],norm[2],norm[3])
         local dot = torch.dot(norm,dir)
         if torch.abs(dot - dots[fi][di]) > 1e-8 then
            errstr = errstr .. "\n-- dot product error"
            dperr = dperr + 1
            errp = true
         end
         if torch.abs(1/dot - invdots[fi][di]) > 1e-8 then
            errstr = errstr .. "\n-- inv dot error"
            idoterr = idoterr + 1
         end
         local t = -( torch.dot(norm,pt) + d)
         if torch.abs(t - ts[fi]) > 1e-8 then
            errstr = errstr .. "\n-- initial t error"
            initerr = initerr + 1
            errp = true
         end
         local tdot = t/dot
         local dst = dists[fi][di]
         local derr = torch.abs(tdot - dst)
         -- error relative to magnitude 
         if derr/tdot > 1e-8 then
            errstr = errstr .. 
               string.format("\n-- distance error %f <-> %f",tdot,dst)
            finerr = finerr + 1
            errp = true
         end
         local dist = distances[fi][di]
         local i,rpt = 
            geom.ray_plane_intersection(pt,dir,norm,d)
         if i then
            -- relative error
            local err = torch.abs(rpt - dist)
            if err/rpt > 1e-8 then
               nerr = nerr + 1
               errstr = errstr .. 
                  string.format("-- %2.2f <-> %2.2f err: %2.2f", rpt,dist,err)
               errp = true
            end
         end
         if errp then print(errstr) end
      end
   end
   local ntests = norms:size(1) * dirs:size(1)
   printf("-- Errors ( in %2.2fs) ", log.toc())
   printf("-- Dot            : %d/%d", dperr, ntests)
   printf("-- Inv Dot        : %d/%d", idoterr, ntests)
   printf("-- Initial t      : %d/%d", initerr, ntests)
   printf("-- by hand dists  : %d/%d", finerr, ntests)
   printf("-- geom.ray_plane : %d/%d", nerr, ntests)
end

function test_ray_face_intersection ()
   print("Testing ray face intersections")
   local pi    = 1
   dirs  = grid_contiguous(compute_dirs(poses,pi,scale),32,32)
   local obj   = target
   local err   = 0
   local norms = obj.normals
   local d     = obj.d
   local pt    = poses.xyz[pi] 
   local face_verts = obj.face_verts
   local nverts_per_face  = obj.nverts_per_face

   -- precompute the indexes of the dominant dimensions for each face 
   local nfaces = face_verts:size(1)
   local most_planar_normals = torch.IntTensor(nfaces,2)
   for i = 1,nfaces do
      local _,ds = torch.sort(torch.abs(norms[i]))
      most_planar_normals[i]:copy(ds:narrow(1,1,2))
   end

   dirs      = dirs[3][4]
   slow_ds   = torch.Tensor(dirs:size(1))
   slow_fids = torch.IntTensor(dirs:size(1))
   log.tic()
 
   for di = 1,dirs:size(1) do
      slow_ds[di],slow_fids[di] = get_occlusions(pt,dirs[di],obj)
   end

   fast_ds = fast_ray_face_intersection(pt,dirs,
                                        norms,d,
                                        face_verts,
                                        most_planar_normals,
                                        nverts_per_face,
                                        slow_ds,slow_fids)   
   local errs = torch.abs(fast_ds - slow_ds)
   local err  = torch.max(errs)
   printf("-- %d/%d Errors",torch.sum(torch.gt(errs,1e-8)),
                       errs:size(1))
end

function test_get_occlusions ()
   local pi    = 40
   local scale = 16
   local dirs  = compute_dirs(poses,pi,scale)
   local obj   = target
   local pt    = poses.xyz[pi] 

   slow_ds   = torch.Tensor(dirs:size(1),dirs:size(2)):fill(-1)
   slow_fids = torch.IntTensor(dirs:size(1),dirs:size(2)):fill(-1)
   
   for i = 1,dirs:size(1) do
      for j = 1,dirs:size(2) do
         slow_ds[i][j],slow_fids[i][j] = get_occlusions(pt,dirs[i][j],obj)
      end
   end
   return slow_ds,slow_fids
   end

function debug_traverse()
   misfaces = {}
   for pi = 1,1 do
      local pose = poses[pi]
      dirs = pose:get_dirs(scale,packetsize)
      -- dirs = dirs:narrow(1,16,32):narrow(2,112,32)
      out_tree    = torch.Tensor(dirs:size(1),dirs:size(2))
      fid_tree    = torch.LongTensor(dirs:size(1),dirs:size(2))
      out_exhaust = torch.Tensor(dirs:size(1),dirs:size(2))
      fid_exhaust = torch.LongTensor(dirs:size(1),dirs:size(2))
      for ri = 1,dirs:size(1) do
         for ci = 1,dirs:size(2) do
            local ray = Ray(pose.xyz,dirs[ri][ci])

            local tree_d, tree_fid = traverse_tree(tree,target,ray)
            out_tree[ri][ci] = tree_d
            fid_tree[ri][ci] = tree_fid


            local slow_d,slow_fid = get_occlusions(ray,target)
            out_exhaust[ri][ci] = slow_d
            fid_exhaust[ri][ci] = slow_fid
            if slow_d == math.huge then
               printf("max depth exhaust: %f", slow_d)
               -- rerun with debugging
               slow_d,slow_fid = get_occlusions(ray,target,true)
            end

            if (tree_fid ~= slow_fid) or (tree_d ~= slow_d) then
               printf("[%d][%d] tree: %f,%d exhaustive: %f, %d",
                      ri,ci,tree_d,tree_fid, slow_d,slow_fid)
               if misfaces[slow_fid] then
                  misfaces[slow_fid] = misfaces[slow_fid] + 1
               else
                  misfaces[slow_fid] = 1
                  -- traverse tree first time the face isn't found
                  traverse_tree(tree,target,ray,true)
               end
            end
         end
      end
      image.display{image={out_tree,out_exhaust},min=0,max=5}
   end
end
