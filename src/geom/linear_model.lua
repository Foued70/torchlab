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
