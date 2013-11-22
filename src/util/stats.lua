Class()

function remove_mean (pts,dim,m)
   dim = dim or 1
   local d,n
   if dim == 1 then    
      d    = pts:size(dim)
      n    = pts:nElement()/d
      pts  = pts:clone():reshape(d,n)
      m    = m or pts:mean(2)
      m:resize(m:nElement(),1)
   elseif dim == pts:nDimension() then
      d    = pts:size(dim)
      n    = pts:nElement()/d
      pts  = pts:clone():reshape(n,d)
      m    = m or pts:mean(1)
      m:resize(1,m:nElement())
   end
   pts:add(-1,m:expandAs(pts))
   return pts, m:squeeze()
end

-- TODO some other kernels
function gaussian_kernel(x,c)
   c = c or 1
   return torch.cmul(x,x):mul(-c):exp()
end


-- expects points DxN
function covariance(pts,m)
   local d = pts:size(1)
   local n = pts:size(2)
   local pts,m = remove_mean(pts, 1, m)
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
function weighted_covariance(pts, w, m)
   local d = pts:size(1)
   local pts,m = remove_mean(pts, 1, m)
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

-- expects NxD for svd
function min_pca_svd (pts, d)
   d = d or pts:size(pts:nDimension()) -- TODO allow other dimensions
   local n = pts:nElement()/d
   pts = pts:reshape(n,d)
   -- svd expects pts NxD which is opposite of what we usually want
   _,_,v = torch.svd(pts)
   return v:t()[d]
end

-- using eig requires a symmetric matrix such as a covariance matrix
function min_pca_eig(c)
   local e,v = torch.eig(c,'V')
   local _,i = e[{{},1}]:min(1)
   return v:t()[i]
end

-- TODO mean_shift, weighted_mean
-- function mean_shift (x,m,kernel) end
