Class()

-- Like unfold but produces contiguous chunks which would replicate
-- data in overlaps.  Start with a less general version which takes
-- h,w,dims matrix as input and outputs r,c,s1*s2,dims as
-- output. (Contiguous unfold useful for bilateral filtering also).
function unfold_contiguous (m,s1,s2)
   local uf = m:unfold(1,s1,s1):unfold(2,s2,s2)
   local out = torch.Tensor(uf:size(1),uf:size(2),uf:size(4)*uf:size(5),uf:size(3))
   for r = 1,uf:size(1) do
      for c = 1,uf:size(2) do
         out[r][c]:copy(uf[r][c])
      end
   end
   return out
end


-- <vals> sorted list of values
--
-- <val> value for which we want pointer into list where everything
--       before pointer is < val and after >= val

function get_index_lt_val(vals,val)
   return vals:lt(val):sum()
end


-- takes a LongStorage <size> and adds a new dimension in the first dimension. eg: 
-- 
--   > t:size()
-- 
--    3280
--    4948
--    [torch.LongStorage of size 2]
-- 
--   > add_slices(3,t:size())
-- 
--    3
--    3280
--    4948
--    [torch.LongStorage of size 3]
-- 
function add_slices(n_slices, size)
   local out_size = {n_slices} 
   for i = 1,size:size() do table.insert(out_size, size[i]) end
   return torch.LongTensor(out_size):storage()
end

-- Really need a FILE GLOB...
function file_match(dir,match) 
   if not paths.dirp(dir) then return nil end
   out = {}
   for f in paths.files(dir) do 
      if f:gmatch(match)() then
         table.insert(out,dir .. "/" .. f) 
      end 
   end
   return out
end
