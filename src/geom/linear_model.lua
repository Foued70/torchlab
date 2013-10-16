Class()

function fit(pts)
   pts = pts:clone()
   -- pts Nx3
   -- center
   m = pts:mean(1)
   pts:add(-1,m:expandAs(pts))
   u,s,v = torch.svd(pts)

   n = v:t()[3]:clone()
   d = - (n * m:squeeze())
   return n,d
end

function residual(pts,n,d)
   -- pts is Nx3
   res = torch.mv(pts,n)
   res:add(d):abs()
   return res
end
