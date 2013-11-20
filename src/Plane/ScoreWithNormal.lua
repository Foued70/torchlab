local compute_residual = geom.linear_model.residual_fast
local cosine_distance  = Plane.util.cosine_distance
local sweep_threshold  = Plane.util.sweep_threshold
local select_points    = Plane.util.select_points_fast

local ScoreWithNormal = Class()

function ScoreWithNormal:__init(...)
   _,
   self.max_distance_from_plane,
   self.max_radians_from_normal,
   self.n_measurements = 
      dok.unpack(
         {...},
         'ScoreWithNormal',
         'Score the plane with points that fall within a threshold',
         {arg='max_distance_from_plane',
          type='number',
          help='max distance of points from the plane used for scoring',
          default=nil},
         {arg='max_radians_from_normal',
          type='number',
          help='max radians distance from plane normal to accept point as part of plane',
          default=math.pi/3},
         {arg='n_measurements',
          type='number',
          help='number of measurements to take when computing area under the curve',
          default=10})

   self.residual        = torch.Tensor()
   self.normal_distance = torch.Tensor()
end

function ScoreWithNormal:compute(plane_eqn, points, normals)
   local normal_threshold = self.max_radians_from_normal
   local normal_distance  = self.normal_distance
   cosine_distance(plane_eqn, normals, normal_distance)
   local normal_mask     = normal_distance:lt(normal_threshold)
   local points_filtered = select_points(points,normal_mask)

   local residual = self.residual

   compute_residual(plane_eqn,points_filtered, residual)
   residual:abs()

   local max_residual = self.max_distance_from_plane or residual:max()
   local n_measurements = self.n_measurements


   local curve = sweep_threshold(residual, max_residual, n_measurements)
   local n_pts = curve[2][n_measurements]
   local score = curve[2]:sum()/(n_pts*n_measurements) -- Area under curve

   self.score = score
   self.curve = curve
   self.n_pts = n_pts

   return score, curve, n_pts
end
