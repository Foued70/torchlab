local linear_fit = geom.linear_model.fit
Class()

function fit_plane_to_points(pts, m)
   local plane_eqn, m = linear_fit(pts, m)
   if plane_eqn[4] > 0 then 
      plane_eqn:mul(-1)
   end
   return plane_eqn, m
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



