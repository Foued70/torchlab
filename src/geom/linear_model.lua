-- funcs from stats
local remove_mean         = util.stats.remove_mean
local min_pca_svd         = util.stats.min_pca_svd
local min_pca_eig         = util.stats.min_pca_eig
local covariance          = util.stats.covariance
local weighted_covariance = util.stats.weighted_covariance

Class()
-- compute weights + bias 
-- eg: fit a plane : ax + by + cz + d = 0 

function fit(pts)
   -- pts NxD (D == 3 for 3D points)
   local d        = pts:size(2)
   local pts,m    = remove_mean(pts,2) 
   local n        = min_pca_svd(pts)
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end

function fit_eig (pts)
   local c,m      = covariance(pts)
   local d        = c:size(1)
   local n        = min_pca_eig(c)
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end

function fit_weighted(pts, w)
   w = w or torch.ones(pts[1]:size()):div(pts[1]:nElement())
   local c,m      = weighted_covariance(pts,w)
   local d        = c:size(1)
   local n        = min_pca_eig(c)
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end

function normal_towards_origin(model)
   -- check if bias is positive
   if model[model:size(1)] > 0 then 
      model:mul(-1)
   end
   return model
end

function residual(model, pts, res)
   d = pts:size(2)
   if d ~= model:size(1)-1 then 
      error("model size "..model:size(1).." not eq pts dim "..pts:size(2))
   end
   res = res or torch.Tensor()
   res:resize(pts:size(1))
   -- pts is Nx3
   torch.mv(res,pts,model[{{1,d}}])
   res:add(model[d+1])
   return res
end

-- pts is DxN as should be
function residual_fast(model, pts, res)
   d = pts:size(1)
   if d ~= model:size(1)-1 then 
      error("model size "..model:size(1).." not eq pts dim "..pts:size(2))
   end
   res = res or torch.Tensor()
   res:resize(pts:size(2)):copy(pts[1]):mul(model[1])
   for i = 2,d do 
      res:add(torch.mul(pts[i],model[i]))
   end
   res:add(model[d+1])
   return res
end

