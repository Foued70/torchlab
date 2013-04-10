Class()

-- Like unfold but produces contiguous chunks which would replicate
-- data in overlaps.  Start with a less general version which takes
-- h,w,dims matrix as input and outputs r,c,s1*s2,dims as
-- output. (Contiguous unfold useful for bilateral filtering also).
function grid_contiguous (m,s1,s2)
   local uf = m:unfold(1,s1,s1):unfold(2,s2,s2)
   local out = torch.Tensor(uf:size(1),uf:size(2),uf:size(4)*uf:size(5),uf:size(3))
   for r = 1,uf:size(1) do
      for c = 1,uf:size(2) do
         out[r][c]:copy(uf[r][c])
      end
   end
   return out
end

-- need C code for this

function select_by_index(ind,mat)
   local nelem = ind:size(1)
   -- allow for any dimension as long as we index by the first
   local shape = mat:size()
   shape[1] = nelem

   local s = torch.Tensor(shape)
   for i = 1,nelem do
      s[i] = mat[ind[i]]
   end
   return s
end

-- <vals> sorted list of values
-- 
-- <val> value for which we want pointer into list where everything
--       before pointer is < val and after >= val

function get_index_lt_val(vals,val)
   local idx  = math.ceil(vals:size(1)*0.5)
   local cval = vals[idx]
   local pval = vals[idx-1]
   local vals_size = vals:size(1)
   local step = math.floor(idx*0.5)
   local count = 0
   -- O(n) code for debugging
   local debug = false
   local onidx = idx
   if debug then
      printf("size: %d idx: %f pval: %f cval: %f step: %f",vals_size,idx,pval,cval,step)
      for i = 1,vals_size do
         if (vals[i] >= val) then
            onidx = i
            break
         end
      end
   end
   if val <= torch.min(vals) then 
      return 1,0 
   end
   if val >= torch.max(vals) then 
      return vals:size(1),0 
   end
   -- O(log(n)) code which had some edge cases
   while true do
      count = count + 1
      -- stop condition or take step
      if ((cval >= val) and (pval < val)) then
         break            -- stop condition
      elseif (cval < val) then
         idx = idx + step -- step forwards
      else
         idx = idx - step -- step backwards
      end
      -- only decrement if step is > 1
      if (step > 1) then
         step = math.floor(0.5 + (step * 0.5))
      end
      -- keep things inbounds
      if (idx < 2) then idx = 2 end
      if (idx > vals_size) then idx = vals_size end
      -- setup next loop
      cval = vals[idx]
      pval = vals[idx-1]
   end
   if debug and (onidx ~= idx) then
      printf("found: %f %f %d", idx, onidx, count)
      printf("sv: %f on[%f]: %f,%f",val, onidx, vals[onidx-1], vals[onidx])
      printf("          log[%f]: %f,%f", idx, vals[idx-1],vals[idx])
      print(vals)
      error("dont match")
   end
   return idx,count
end