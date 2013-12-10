local select_points = Plane.util.select_points_fast
local fit_plane_to_points = Plane.util.fit_plane_to_points

local Validate = Class()

function Validate:__init(...)
   _,
   self.residual_threshold, 
   self.normal_threshold, 
   self.min_points_for_seed, 
   self.min_points_for_plane,
   self.min_auc_score = 
      dok.unpack(
         {...},
         'Validate',
         'validate a plane in a set of points given a subset of points',
         {arg='residual_threshold',
          type='number',
          help='minimum score (std of residual) to accept a plane',
          default=40},
         {arg='normal_threshold',
          type='number',
          help='minimum score (std of residual) to accept a plane',
          default=math.pi/3},
         {arg='min_points_for_seed',
          type='number',
          help='minimum number of points to consider set at seed',
          default=150},
         {arg='min_points_for_plane',
          type='number',
          help='minimum number of points to consider set a plane',
          default=900},
         {arg='min_auc_score',
          type='number',
          help='minimum area under the curve to consider seed a plane',
          default=0.5}
      )

   self.score = Plane.ScoreWithNormal.new{
      max_distance_from_plane = self.residual_threshold,
      max_radians_from_normal = self.normal_threshold
   }
   self.use_slope_score = false
end

function Validate:seed(points, normals, mask, debug_info)
   
   local current_plane = nil
   local error_string  = nil

   -- block for finding planes based on seed or from support of expanded set
   while true do -- this is a dummy block to replace a string of nested ifs with breaks.
      local seed_n_pts = mask:sum()
      local mask_size  = mask:size()
      -- 1) 1st test: seed has a minimum number of points to form a plane
      if ( seed_n_pts < self.min_points_for_seed) then
         error_string="not enough seed points"
         break
      end

      local seed_pts = select_points(points,mask)

      local seed_plane = Plane.Plane.new{
         residual_threshold = self.residual_threshold,
         normal_threshold   = self.normal_threshold
      }
      seed_plane.eqn, seed_plane.center = fit_plane_to_points(seed_pts)
      local seed_score  = seed_plane:score(seed_pts)

      -- 2) 2nd test: points in the segment explain a plane (no outliers in the segment)
      if (seed_score > self.residual_threshold) then
         error_string="seed plane above threshold"
         break
      end

      local seed_explained_pts, seed_explained_n_pts, seed_explained_mask =
         seed_plane:filter_points(points, normals)
      
      -- 3) 3rd test: seed plane explains the minimum number of points.      
      if seed_explained_n_pts < self.min_points_for_seed then
         error_string="seed explained points (after filtering) do not explain enough points for seed"
         break
      end
      
      local seed_explained_score = seed_plane:score(seed_explained_pts)
      
      seed_plane.auc_score, seed_plane.curve = 
         self.score:compute(seed_plane.eqn, points, normals, self.use_slope_score)
      
      -- 4) 4th test if seed explains oints are below threshold we have a plane
      if (seed_explained_score > self.residual_threshold) then
         error_string='points explained by seed not above residual'
         break
      end

      -- 5) 5th test if curve of points is above minimum area under curve score
      if (seed_plane.auc_score < self.min_auc_score) then
         error_string='auc_score below threshold'
         break
      end

      -- the plane from original seed that explains a minumum number of points
      current_plane = seed_plane
      error_string  = "Found plane for seed."
      break -- break out of dummy loop for plane finding
   end 
   return current_plane, error_string
end
