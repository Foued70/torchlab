local util = {}

-- Like unfold but produces contiguous chunks which would replicate
-- data in overlaps.  Start with a less general version which takes
-- h,w,dims matrix as input and outputs r,c,s1*s2,dims as
-- output. (Contiguous unfold useful for bilateral filtering also).
function util.grid_contiguous (m,s1,s2)
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

function util.get_index_lt_val(vals,val)
   local idx = vals:size(1)*0.5
   local cval = vals[idx]
   local nval = vals[idx+1]
   local step = idx*0.5
   local count = 0
   while true do 
      count = count + 1
      if (cval >= val) then
         idx = idx - step
      elseif (nval < val) then
         idx = idx + step
      else
         break
      end
      step = math.floor(0.5 + (step * 0.5))
      if (idx <= 0) or (idx >= vals:size(1)) or (step < 1) then
         break
      end
      cval = vals[idx]
      nval = vals[idx+1]
   end
   if (idx < 1) or (idx >= vals:size(1)) then
      return -1,count
   end

   return math.floor(idx),count
end


return util