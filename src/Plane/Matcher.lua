local cosine_distance       = Plane.util.cosine_distance
local residual              = geom.linear_model.residual_fast
local kernel                = util.stats.gaussian_kernel

local Matcher = Class()
function Matcher:__init(...)
   _,
   self.residual_threshold,
   self.normal_threshold =
      dok.unpack(
         {...},
         'Matcher',
         'Matches sets of oriented points (points, normals) with a list of planes',
         {arg='residual_threshold',
          type='number',
          help='distance beyond which a point is considered not to belong to the plane',
          default=100},
         {arg='normal_threshold',
          type='number',
          help='radians beyond which a point normal is considered not to belong to the plane',
          default=math.pi/2})

end

-- points and normals in DxN format
function Matcher:match(planes, points, normals)
   -- TODO could do this with less memory
   local score = self.score or torch.Tensor()
   score:resize(#planes,points:size(2))
   
   -- loop through planes
   for i,p in pairs(planes) do 
      -- TODO if not plane fit plane w/ warning.
      local plane_eqn    = p.eqn
      local residuals    = residual(       plane_eqn, points)
      local normal_dists = cosine_distance(plane_eqn, normals)
      
      local residual_weights = kernel(torch.div(residuals,self.residual_threshold))
      local normal_weights   = kernel(torch.div(normal_dists,self.normal_threshold))
      -- TODO some mixing coeff ? alpha = 0.5
      score[i]:copy(torch.cmul(residual_weights,normal_weights))
   
   end

   return score:max(1)
end
