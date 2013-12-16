local cosine_distance       = Plane.util.cosine_distance
local residual              = geom.linear_model.residual_fast
local kernel                = util.stats.gaussian_kernel
local path = require "path"
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
   local d = points:size(1)
   local n = points:nElement()/d
   points = points:reshape(d,n)
   normals = normals:reshape(d,n)
   -- TODO could do this with less memory
  score = self.score or torch.Tensor()
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

-- points and normals in DxN format
function Matcher:matchDistance(pc, planes)
  local points = pc:get_xyz_map()
  local normals, temp, temp, temp, maskN = pc:get_normal_map()
  local d = points:size(1)
  local size_orig = points:size()
  local points = points:reshape(3,points:size(2)*points:size(3))
  local normals = normals:reshape(3,normals:size(2)*normals:size(3))
  local n = points:nElement()/d
  local points = points:reshape(d,n)
  local normals = normals:reshape(d,n)
   -- TODO could do this with less memory
  local score =  torch.Tensor()
  score:resize(#planes,points:size(2))

  --TO DO this can be saved at plane creation w real location
  local center_loc = torch.zeros(table.getn(planes))
  for i=1, table.getn(planes) do
    local temp,loc = (points-planes[i].center:repeatTensor(1,points:size(2))):norm(2,1):squeeze():min(1)
    center_loc[i] = loc[1]
  end
  for i,p in pairs(planes) do
    local plane_eqn    = p.eqn
    local residuals    = residual(       plane_eqn, points)
    local normal_dists = cosine_distance(plane_eqn, normals)

    local temp = residuals:reshape(size_orig[2], size_orig[3])
    temp[torch.lt(temp,0)] = 0
    temp[maskN] = 0
    local t = boost_graph.Graph.shortest_path(temp:clone():log1p(), center_loc[i])
    
    local residual_weights = kernel(torch.div(residuals,self.residual_threshold))
    local normal_weights   = kernel(torch.div(normal_dists,self.normal_threshold))
    local dis_weights = kernel(torch.div(t:reshape(size_orig[2]*size_orig[3]),300))
    score[i]:copy(torch.cmul(residual_weights,normal_weights):cmul(dis_weights))   
    collectgarbage()
  end
  score,ord = score:max(1)
  ord = ord:squeeze():reshape(size_orig[2], size_orig[3])
  score = score:squeeze():reshape(size_orig[2], size_orig[3])

  return score, ord
end

function Matcher.removeNoise(score, ord, planes, points, threshold)
   -- loop through planes
   threshold = threshold or .7
  local new_ord = ord:clone()
  for i,p in pairs(planes) do 
    local reasonable = torch.eq(new_ord,i)
    new_ord[reasonable:clone():cmul(torch.lt(score, threshold))]=0
    reasonable:cmul(torch.ge(score,threshold))
    -- TODO if not plane fit plane w/ warning.
    local ccs = Plane.Matcher.getCCs(reasonable:double():clone())
    if ccs then
      new_ord[reasonable:clone():cmul(torch.gt(ccs:sum(1):squeeze(),0)*-1+1)] = 0
      for j=1, ccs:size(1) do
        new_pts = flattened_plane.FlattenedPlane.select3d(points, ccs[j])
        res = residual(p.eqn, new_pts:t())
        if(math.abs(res:mean())>15 or math.abs(res:std())>25) then
          new_ord[ccs[j]] = 0
        end
      end
      collectgarbage()
    else
      new_ord[reasonable] = 0
    end
  end
  return new_ord
end
function Matcher.getCCs(image)
    local imgraph = require "../imgraph/init.lua"
    local graph = imgraph.graph(image,8)
    local cc = imgraph.connectcomponents(graph, 0.8, false)
    local cca = imgraph.adjacency(cc)
    local good_ks={}
    local cost = {}
    counter = 1
    for k,v in pairs(cca) do
        local totsO = torch.eq(cc,k):sum()
        if(totsO > 50 and torch.eq(cc,k):cmul(image:byte()):sum()/totsO > .5) then
            good_ks[counter] = k
            cost[counter] = torch.eq(cc,k):sum()
            counter = counter+1
        end
    end
    if(counter == 1) then
      return nil
    end
    good_ks = torch.Tensor(good_ks)
    cost = torch.Tensor(cost)
    a,order = torch.sort(cost, true)
    local ccs = torch.zeros(good_ks:size(1), image:size(1), image:size(2)):byte()
    for i =1, good_ks:size(1) do
       ccs[i] = torch.eq(cc,good_ks[order[i]])
    end
    return ccs
end