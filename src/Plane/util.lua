local linear_fit = geom.linear_model.fit
Class()

-- Plane from single point and normal 
function plane_from_point( point, normal )
   local plane_eqn = torch.Tensor(4)
   plane_eqn[{{1,3},{}}] = normal 
   plane_eqn[{4}] = -torch.dot(point, normal)
   if plane_eqn[4] > 0 then 
      plane_eqn:mul(-1)
   end
   return plane_eqn
end

function fit_plane_to_points(pts, m)
   local plane_eqn, m = linear_fit(pts, m)
   if plane_eqn[4] > 0 then 
      plane_eqn:mul(-1)
   end
   return plane_eqn, m
end

-- Transform a plane given a homogenous transformation matrix 
function transform_plane( plane, transform ) 

   local normal = torch.cat(plane.eqn:sub(1,3), torch.Tensor({0}))
   local centroid = torch.cat(plane.centroid, torch.Tensor({1}))  

   -- Transform plane normal and centroid
   local tfd_normal = transform*normal
   local tfd_centroid = transform*centroid

   -- Compute new d for plane eqn 
   local d = torch.Tensor({torch.dot( tfd_normal:sub(1,3), tfd_centroid:sub(1,3) )})

   local tfd_plane = plane
   tfd_plane.eqn = torch.cat( tfd_normal:sub(1,3), d)
   tfd_plane.centroid = tfd_centroid:sub(1,3)
   return tfd_plane
end


-- torch doesn't have a determinant operation, how lame is that
-- Input 3x3 matrix 
-- TODO: this shouldn't really be placed here
function det3x3( matrix )
   return matrix[{{1},{1}}]*matrix[{{2},{2}}]*matrix[{{3},{3}}] + 
          matrix[{{1},{2}}]*matrix[{{2},{3}}]*matrix[{{3},{1}}] + 
          matrix[{{1},{3}}]*matrix[{{2},{1}}]*matrix[{{3},{2}}] - 
          matrix[{{1},{3}}]*matrix[{{2},{2}}]*matrix[{{3},{1}}] - 
          matrix[{{1},{2}}]*matrix[{{2},{1}}]*matrix[{{3},{3}}] - 
          matrix[{{1},{1}}]*matrix[{{2},{3}}]*matrix[{{3},{2}}]
end

function estimate_transform_from_planes( tfd_planes, planes )   
   local normals = torch.Tensor(3,#planes)
   for i = 1,#planes do 
      --normals[{{},{i}}] = planes[i].eqn:sub(1,3)
      normals[{{},{i}}] = planes[i].eqn:sub(1,3)
   end

   local tfd_normals = torch.Tensor(3,#tfd_planes)
   for i = 1,#tfd_planes do 
      tfd_normals[{{},{i}}] = tfd_planes[i].eqn:sub(1,3)
   end

   local d_errs = torch.Tensor(#planes)
   for i = 1,#planes do 
      d_errs[{i}] = tfd_planes[i].eqn[{4}] - planes[i].eqn[{4}]
   end

   -- Compute rotation
   local U,S,V = torch.svd(tfd_normals*normals:t())

   local R = U*V:t()   
   -- Check for reflection and correct
   if det3x3(R):squeeze() < 0 then 
      R[{{},{3}}] = R[{{},{3}}]:mul(-1)
   end
   -- TODO: check for degeneracy and what-not

   -- Compute translation
   local T = torch.inverse(tfd_normals:t())*d_errs

   local tf = torch.eye(4)
   tf[{{1,3},{1,3}}] = R
   tf[{{1,3},{4}}] = -T
   return tf 
end

function sweep_threshold(distances, max_dist, n_measurements)
   max_dist = max_dist or distances:max()
   n_measurements = n_measurements or 10
   local cumulative = torch.Tensor(2,n_measurements)
   local r = torch.linspace(1,max_dist,n_measurements)
   cumulative[1] = r
   for j = 1,n_measurements do 
      local thres = r[j]
      cumulative[2][j] = distances:lt(thres):sum()
   end
   return cumulative
end

function sweep_threshold_2d(distances, max_dist, d2, max_d2, n_measurements)
   max_dist = max_dist or distances:max()
   n_measurements = n_measurements or 10
   local cumulative = torch.Tensor(n_measurements,n_measurements)
   local r = torch.Tensor(2,n_measurements)
   r[1] = torch.linspace(1,max_dist,n_measurements)
   r[2] = torch.linspace(math.pi/360,max_d2,n_measurements)
   for i = 1,n_measurements do 
      local thres = r[{1,i}]
      for j = 1,n_measurements do 
         local thres2 = r[{2,j}]
         cumulative[{i,j}] = distances:lt(thres):cmul(d2:lt(thres2)):sum()
      end
   end
   return r,cumulative
end

-- expects DxN
function get_distance(point,points)
   local tmp = points:clone()
   tmp:add(-1,point:reshape(3,1):expandAs(points))
   tmp:cmul(tmp)
   local out = tmp:sum(1)
   return out:sqrt():squeeze()
end

-- expects NxD
function select_points(pts,mask)
   local pts_d  = pts:size(pts:nDimension())
   local n_pts  = mask:sum()
   local mask   = mask:reshape(mask:nElement(),1):expandAs(pts)
   local outpts = pts[mask]
   return outpts:resize(n_pts,ptsd), n_pts
end

-- expects DxN
function select_points_fast(pts,mask)
   local d      = pts:size(1)
   pts = pts:reshape(d,pts:nElement()/d)
   local n_pts  = mask:sum()
   local mask   = mask:reshape(1,mask:nElement()):expandAs(pts)
   local outpts = pts[mask]:resize(d,n_pts)
   return outpts, n_pts
end

function score_plane(plane_eqn, points, max_dist, n_measurements)
   local residual = geom.linear_model.residual(points,plane_eqn):abs()
   max_dist = max_dist or residual:max()
   n_measurements = n_measurements or 10
   local curve = sweep_threshold(residual, max_dist, n_measurements)
   local n_pts = curve[2][n_measurements]
   return curve[2]:sum()/(n_pts*n_measurements), curve, n_pts
end

function score_plane_with_normal_threshold(plane_eqn, points, normals, max_dist,  normal_thres, n_measurements)
   local cosine_dist = Plane.util.cosine_distance(plane_eqn,normals)
   max_dist = max_dist or residual:max()
   normal_thres = normal_thres or math.pi/6
   n_measurements = n_measurements or 10
   normal_mask = cosine_dist:lt(normal_thres)
   points_filtered = select_points_fast(points,normal_mask)
   local residual = geom.linear_model.residual_fast(plane_eqn,points_filtered):abs()
   local curve = sweep_threshold(residual, max_dist, n_measurements)
   local n_pts = curve[2][n_measurements]
   return curve[2]:sum()/(n_pts*n_measurements), curve, n_pts
end

function score_plane_2D(plane_eqn, points, normals, max_dist, max_normal_dist, n_measurements)
   local cosine_dist = Plane.util.cosine_distance(plane_eqn,normals)
   local residual    = geom.linear_model.residual_fast(plane_eqn,points):abs()
   max_dist = max_dist or residual:max()
   n_measurements = n_measurements or 10
   local xaxis, curves = sweep_threshold_2d(residual, max_dist, cosine_dist, max_normal_dist, n_measurements)
   local n_pts = curves:max()
   return curves:sum()/(n_pts*curves:nElement()), xaxis, curves, n_pts
end

-- normals dim 2 size (N,3)
-- plane_eqn dim 1 size 4
function cosine_distance(plane_eqn, normals, result)
   local d = normals:size(1)
   result = result or torch.Tensor()
   result:resizeAs(normals[1]):copy(normals[1]):mul(plane_eqn[1])
   for i = 2,d do 
      result:add(torch.mul(normals[i],plane_eqn[i]))
   end
   result:acos()
   return result
end



