Class()

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

function score_plane(plane_eqn, points, max_dist, n_measurements)
   local residual = geom.linear_model.residual(points,plane_eqn):abs()
   max_dist = max_dist or residual:max()
   n_measurements = n_measurements or 10
   local curve = sweep_threshold(residual, max_dist, n_measurements)
   return curve[2]:sum()/(curve[2][n_measurements]*n_measurements), curve
end

function compute_normal_diff(plane_eqn, normals)
   local diff = normals - plane_eqn[{{1,3}}]:resize(1,3):expandAs(normals)
   return diff:abs():sum(2)
end

function compute_normal_dist(plane_eqn, normals)
   local diff = normals:clone()
   diff:add(-1,plane_eqn[{{1,3}}]:resize(1,3):expandAs(normals))
   return diff:cmul(diff):sum(2):squeeze():sqrt()
end


