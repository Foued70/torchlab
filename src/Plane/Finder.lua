fit_plane     = Plane.finder_utils.fit_plane
score         = Plane.finder_utils.score
select_points = Plane.finder_utils.select_points
fpep          = Plane.finder_utils.find_points_explained_by_plane

local pi = math.pi
local Finder = Class()

function Finder:__init(...)
   _,
   self.threshold, 
   self.min_points_for_seed, 
   self.min_points_for_plane,
   self.normal_filter,
   self.normal_threshold = 
      dok.unpack(
         {...},
         'Finder',
         'validate a plane in a set of points given a subset of points',
         {arg='threshold',
          type='number',
          help='minimum score (std of residual) to accept a plane',
          default=40},
         {arg='min_points_for_seed',
          type='number',
          help='minimum number of points to consider set at seed',
          default=150},
         {arg='min_points_for_plane',
          type='number',
          help='minimum number of points to consider set a plane',
          default=900},
         {arg='normal_filter',
          type="boolean",
          help="should filter by point normals",
          default=true},
         {arg='normal_threshold',
          type='number',
          help='minimum difference in point normals to accept as part of same plane',
          default=pi/6}
      )
end

function Finder:find_points_explained_by_plane(points, normals, plane_eqn)
   return fpep(points, plane_eqn, self.threshold, 
               self.normal_filter, self.normal_threshold, normals)
end

function Finder:validate_seed(points, normals, mask, debug_info)
   
   local current_plane = nil
   local error_string  = nil

   -- block for finding planes based on seed or from support of expanded set
   while true do -- this is a dummy block to avoid a string of nested ifs with breaks.
      local seed_n_pts = mask:sum()
      local mask_size  = mask:size()
      -- 1) 1st test: seed has a minimum number of points to form a plane
      if ( seed_n_pts < 3) then
         error_string="not enough seed points"
         break
      end

      local seed_pts = select_points(points,mask)

      local seed_plane_eqn = fit_plane(seed_pts)
      local seed_score     = score(seed_plane_eqn,seed_pts)

      -- 2) 2nd test: points in the segment explain a plane (no outliers in the segment)
      if (seed_score > self.threshold) then
         error_string="seed plane above threshold"
         break
      end

      local seed_explained_pts, seed_explained_n_pts, seed_explained_mask =
         self:find_points_explained_by_plane(points, normals, seed_plane_eqn)
      
      -- 3) 3rd test: seed plane explains the minimum number of points. eg. could be 0 after an erosion
      
      if seed_explained_n_pts < self.min_points_for_seed then
         error_string="seed explained points (after filtering) do not explain enough points for seed"
         break
      end
      
      local seed_explained_score = score(seed_plane_eqn,seed_explained_pts)

      -- 4) 4th test if seed explains enough points and points are below threshold we have a plane
      if (seed_n_pts > self.min_points_for_seed) and (seed_explained_score < self.threshold) then
         -- the plane from original seed that explains a minumum number of points
         current_plane =
            {eqn         = seed_plane_eqn,
             n_pts       = seed_explained_n_pts,
             score       = seed_explained_score,
             mask        = seed_explained_mask:resize(mask_size),
             debug_info  = debug_info
            }
         error_string = "Found plane for seed."
      end
      break
   end -- break out of dummy loop for plane finding
   return current_plane, error_string
end
