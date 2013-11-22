local compute_residual = geom.linear_model.residual_fast
local select_points    = Plane.util.select_points_fast
local cosine_distance  = Plane.util.cosine_distance

local Plane = Class()

local count = 1

function Plane:__init(...)
   _,
   self.eqn,
   self.res_thres,
   self.norm_thres =
      dok.unpack(
         {...},
         'Plane',
         'Plane equation and means to score and compute distance to points',
         {arg='eqn',
          type='torch.Tensor',
          help='plane equation (a,b,c,d): torch.Tensor of dimension 4'},
         {arg='residual_threshold',
          type='number',
          help='distance beyond which a point is considered not to belong to the plane',
          default=10},
         {arg='normal_threshold',
          type='number',
          help='radians beyond which a point normal is considered not to belong to the plane',
          default=math.pi/3})
end

function Plane:residual_threshold(thres)
   if thres then 
      self.res_thres = thres
   end
   return self.res_thres
end

function Plane:normal_threshold(thres)
   if thres then 
      self.norm_thres = thres
   end
   return self.norm_thres
end

function Plane:residual(points)
   return compute_residual(self.eqn,points)
end

function Plane:normal_distance(normals)
   return cosine_distance(self.eqn, normals)
end

function Plane:filter_points(points, normals)
   local res = self:residual(points):abs()
   local mask = res:lt(self.res_thres)
   if self.verbose then 
      printf("residual: min: %f max: %f", res:min(), res:max())
   end
   if self.save_images then 
      image.save("mask_residual_"..count..".jpg",image.combine(res:reshape(points[1]:size())))
      image.save("mask_"..count.."_residual.jpg",image.combine(mask:reshape(points[1]:size())))
   end
   if normals then 
      mask:cmul(self:normal_distance(normals):lt(self.norm_thres))
      if self.save_images then 
         image.save("mask_"..count.."_residual+normal.jpg",image.combine(mask:reshape(points[1]:size())))
      end
   end
   count = count + 1
   local pts, n_pts = select_points(points,mask)
   return pts, n_pts, mask
end


-- TODO integrate score class
function Plane:score(points)
   return self:residual(points):std()
end
