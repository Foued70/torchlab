Class()
-- compute weights + bias 
-- eg: fit a plane : ax + by + cz + d = 0 

function fit(pts)
   -- pts NxD (D == 3 for 3D points)
   local d = pts:size(2)
   local pts,m = remove_mean(pts) 
   local n = util.stats.min_pca_svd(pts)
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

function residual(pts,model)
   d = pts:size(2)
   if d ~= model:size(1)-1 then 
      error("model size "..model:size(1).." not eq pts dim "..pts:size(2))
   end
   -- pts is Nx3
   res = torch.mv(pts,model[{{1,d}}])
   res:add(model[d+1])
   return res
end

-- pts is DxN as should be
function residual_fast(pts,model)
   d = pts:size(1)
   if d ~= model:size(1)-1 then 
      error("model size "..model:size(1).." not eq pts dim "..pts:size(2))
   end
   -- pts is Nx3
   res = pts[1]:clone():mul(model[1])
   for i = 2,d do 
      res:add(torch.mul(pts[i],model[i]))
   end
   res:add(model[d+1])
   return res
end

function fit_eig (pts)
   local c,m      = util.stats.covariance(pts)
   local d        = c:size(1)
   local n        = util.stats.min_pca_eig(c)
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end

function fit_weighted(pts, w)
   w = w or torch.ones(pts[1]:size()):div(pts[1]:nElement())
   local c,m      = util.stats.weighted_covariance(pts,w)
   local d        = c:size(1)
   local n        = util.stats.min_pca_eig(c)
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end
