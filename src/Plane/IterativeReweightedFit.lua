local cosine_distance = Plane.util.cosine_distance
local residual        = geom.linear_model.residual_fast
local kernel          = util.stats.gaussian_kernel
local fit_plane       = geom.linear_model.fit_weighted 
local normal_towards_origin = geom.linear_model.normal_towards_origin

local ss              = nn.SoftShrink(0.1)

-- class iterated reweighted   
local itrw = Class()

function itrw:__init(...)
   _,
   self.residual_thres,
   self.residual_decr,
   self.residual_stop,
   self.normal_thres,
   self.normal_decr,
   self.normal_stop,
   self.min_points_for_plane =
      dok.unpack(
         {...},
         'IterativeReweightedFit',
         'find best plane within a threshold using robust iterative reweighting scheme',
         {arg='residual_threshold',
          type='number',
          help='initial largest threshold for distance of points from the plane used for scoring',
          default=40},
         {arg='residual_decr',
          type='number',
          help='percent decrease when reducing residual',
          default=0.7},
         {arg='residual_stop',
          type='number',
          help='minimal distance to plane to stop',
          default=1},
         {arg='normal_threshold',
          type='number',
          help='initial largest angle in radians to consider points part of same plane',
          default=math.pi/6},
         {arg='normal_decr',
          type='number',
          help='percent decrease when reducing normal',
          default=0.7},
         {arg='normal_stop',
          type='number',
          help='minimal angle to consider',
          default=math.pi/360},
         {arg='min_points_for_plane',
          type='number',
          help='minimum number of points to fit ',
          default=150},
         {arg='score_threshold_for_convergence',
          type='number',
          help='score improvement at which we decide that we have convergence ',
          default=1e-4}
      )
   
   self.score = Plane.ScoreWithNormal.new{
      max_distance_from_plane = residual_threshold,
      max_radians_from_normal = normal_threshold
   }
   
   self.save_images = false
   self.image_id    = ''

end

-- TODO score class
-- function itrw:score(plane_eqn, points, normals)
--   return score_plane(
--       plane_eqn , points, normals, self.residual_thres,  self.normal_thres, self.n_measurements)
-- end

function itrw:get_neighborhood(residuals, residual_threshold, normal_dists, normal_threshold)
   local residual_weights = kernel(torch.div(residuals,residual_threshold))
   local normal_weights   = kernel(torch.div(normal_dists,normal_threshold))
      -- TODO some mixing coeff ? alpha = 0.5
   local combo_weights    = torch.cmul(residual_weights,normal_weights)

   -- soft threshold
   local weights          = ss:forward(combo_weights)
   -- hard threshold
   local mask             = weights:gt(0)
   -- number of inliers of hard threshold
   local npts             = mask:sum()
   return weights, mask, npts
end

function itrw:fit(points, normals, plane_eqn)
   local psize = points:size()
   local map_height = points:size(2)
   local map_width  = points:size(3)
   points  = points:reshape(3,points:nElement()/3)
   normals = normals:reshape(3,normals:nElement()/3)
   local current_residual_threshold = self.residual_thres
   local current_normal_threshold   = self.normal_thres

   local best_plane   = plane_eqn
   local residuals    = residual(best_plane,points)
   local normal_dists = cosine_distance(best_plane, normals)

   -- score input plane
   local orig_s, orig_c, orig_n_pts = self.score:compute(best_plane, points, normals)

   -- output for gnuplot
   local curves = {{string.format("iteration: %d thres: %2.1f mm norm: %0.3f rad",
                                  0 , current_residual_threshold,current_normal_threshold),orig_c:t()}}
   local best_s     = orig_s
   local best_n_pts = orig_n_pts
   local doresidual = false
   local new_s = 1
   -- TODO other convergence
   iter = 0
   -- while new_s - best_s > 0.0001 do 
   while iter < 64 do 
      iter = iter + 1
      collectgarbage()
      -- 1) compute mask for points based on current thresholds:
      -- current_residual_threshold and current_normal_threshold

      local weights, mask, npts = self:get_neighborhood(residuals,    current_residual_threshold,
                                                        normal_dists, current_normal_threshold)

      if npts < self.min_points_for_plane then
         printf("only %d points in neighborhood. Stopping",npts)
         break
      else
         -- 2) extract set of points on which to fit.
         filtered_points = torch.Tensor(3,npts)
         for i = 1,3 do 
            filtered_points[i] = points[i][mask] 
         end
         -- 3) compute weights (distance from center of thresholds
         local filtered_weights    = weights[mask]
         -- 4) fit plane to filtered points. includes:
         --    a) remove mean (non-weighted)
         --    b) compute weighted covariance matrix
         --    c) find smallest eigenvector of covariance matrix
         local test_plane = fit_plane(filtered_points, filtered_weights)
         normal_towards_origin(test_plane)
         
         new_s, new_c, new_n_pts = self.score:compute(test_plane, points, normals)
         
         if new_s > best_s then
            printf(" - %s[%d] score old: %f (%d) new: %f (%d) orig: %f (%d)",
                   self.image_id,iter,best_s,best_n_pts,new_s,new_n_pts,orig_s,orig_n_pts)
            table.insert(curves, {string.format("iteration: %d score: %2.4f thres: %2.1f mm, %0.3f rad",
                                                iter, best_s, current_residual_threshold,current_normal_threshold),
                                  new_c:t()})
            print(" - updating plane")
            best_s     = new_s
            best_n_pts = new_n_pts
            best_plane = test_plane
            -- recompute if we change the equation
            residuals    = residual(best_plane, points)
            normal_dists = cosine_distance(best_plane,normals)
            if self.save_images then 
               local inum_str = string.format("%s%03d", self.image_id, iter)
               local result_str = string.format("s%f_r%2.1f_n%0.3f",
                                                new_s,current_residual_threshold,current_normal_threshold)
               weights:resize(map_height,map_width)
               image.save(string.format("%s_%s_neighborhood.png",
                                        inum_str,result_str), image.combine(weights))
            end
         elseif doresidual and (current_residual_threshold > self.residual_stop) then 
            current_residual_threshold = current_residual_threshold * self.residual_decr
            printf("Narrowing nbd : res %f mm", current_residual_threshold)
            doresidual = not doresidual
         elseif (current_normal_threshold > self.normal_stop) then 
            current_normal_threshold = current_normal_threshold * self.normal_decr
            printf("Narrowing nbd : normal %f radians", current_normal_threshold)
            doresidual = not doresidual
         else
            break
         end
      end
   end
   return best_plane, best_s, best_n_pts, curves
end
