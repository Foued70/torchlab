local debug = require 'debug'

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

function search_in_table(t, search_phrase)
  for _,v in pairs(t) do
    if v == search_phrase then
      return true
    end
  end
  return false
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
function locals()
   local variables = {}
  local idx = 1
  while true do
     local ln, lv = debug.getlocal(2, idx)
    if ln ~= nil then
       variables[ln] = lv
    else
      break
    end
    idx = 1 + idx
  end
  return variables
end
