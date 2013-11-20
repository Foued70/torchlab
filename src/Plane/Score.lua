local compute_residual = geom.linear_model.residual_fast
local sweep_threshold  = Plane.finder_utils.sweep_threshold

local Score = Class()

function Score:__init(...)
   _,
   self.max_dist, 
   self.n_measurements = 
      dok.unpack(
         {...},
         'Score',
         'Score the plane with points that fall within a threshold',
         {arg='max_dist',
          type='number',
          help='max distance of points from the plane used for scoring',
          default=nil},
         {arg='n_measurements',
          type='number',
          help='number of measurements to take when computing area under the curve',
          default=10})

   self.residual = torch.Tensor()

end

function Score:compute(plane_eqn, points)
   residual = self.residual

   compute_residual(plane_eqn,points, residual)
   residual:abs()

   max_dist = self.max_dist or residual:max()
   n_measurements = self.n_measurements

   local curve = sweep_threshold(residual, max_dist, n_measurements)
   local n_pts = curve[2][n_measurements]
   local score = curve[2]:sum()/(n_pts*n_measurements) -- Area under curve

   self.score = score
   self.curve = curve
   self.n_pts = n_pts

   return score, curve, n_pts
end
