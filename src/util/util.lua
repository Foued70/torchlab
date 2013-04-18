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
