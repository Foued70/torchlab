Class()

function fit(pts)
   -- pts NxD (D == 3 for 3D points)
   local d = pts:size(2)
   pts = pts:clone()
   -- center
   local m = pts:mean(1)
   pts:add(-1,m:expandAs(pts))
   _,_,v = torch.svd(pts)
   local model    = torch.Tensor(d+1)
   local n        = v:t()[d]
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
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

function remove_mean (pts)
   local d    = pts:size(1)
   local n    = pts:nElement()/d
   local pts  = pts:clone():reshape(d,n)
   local m    = pts:mean(2)
   pts:add(-1,m:expandAs(pts))
   return pts,m
end

-- expects points 3xN
function covariance(pts)
   local pts,m = remove_mean(pts)
   local d = pts:size(1)
   local n = pts:size(2)
   local C = torch.zeros(d,d)
   for i = 1,d do 
      for j = i,d do 
         C[{i,j}] = torch.cmul(pts[i],pts[j]):sum()
         if i ~= j then 
            C[{j,i}] = C[{i,j}]
         end
      end
   end
   return C:div(n),m
end

-- expects points DxN, N weights (one per vector)
function weighted_covariance(pts, w)
   local pts,m = remove_mean(pts)
   local d = pts:size(1)
   local C = torch.zeros(d,d)
   for i = 1,d do 
      for j = i,d do 
         C[{i,j}] = torch.cmul(pts[i],pts[j]):cmul(w):sum()
         if i ~= j then 
            C[{j,i}] = C[{i,j}]
         end
      end
   end
   return C,m
end

function fit_eig (pts)
   local c,m = covariance(pts)
   local d   = c:size(1)
   local e,v = torch.eig(c,'V')
   local _,i = e[{{},1}]:min(1)
   local n = v:t()[i]:clone()
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end

function fit_weighted(pts, w)
   w = w or torch.ones(pts[1]:size()):div(pts[1]:nElement())
   local c,m = weighted_covariance(pts,w)
   local d   = c:size(1)
   local e,v = torch.eig(c,'V')
   local _,i = e[{{},1}]:min(1)
   local n = v:t()[i]:clone()
   local model    = torch.Tensor(d+1)
   model[{{1,d}}] = n
   model[d+1]     = - ( n * m:squeeze())
   return model 
end
