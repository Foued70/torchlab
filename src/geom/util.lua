Class()

function eq(vec1, vec2)
   return torch.min(torch.eq(vec1,vec2)) == 1
end

function direction(vec1, vec2)
   return normalize(vec1 - vec2)
end

function normalize(vec)
   local n = vec:norm()
   if (n > 1e-8) then
      vec:mul(1/n)
   end
   return vec
end

function normalized(...)
   local v,res
   local args = {...}
   local nargs = #args
   if nargs == 2 then
      res = args[1]
      v   = args[2]
   elseif nargs == 1 then
      v   = args[1]
      res = torch.Tensor(v:size())
   else
      print(dok.usage('normalize',
                      'normalize a vector (L2)',
                      '> returns: normalized vector',
                      {type='torch.Tensor', help='output'},
                      {type='torch.Tensor', help='input', req=true}))
      dok.error('incorrect arguments', 'normalize')
   end

   if (res ~= v) then res:copy(v) end
   normalize(res)
   return res
end

-- compute normal from first three vertices in a face
function compute_normal(v)
   return normalize(torch.cross(v[3] - v[2], v[1] - v[2]))
end

function angle_between(vec1, vec2)
   return torch.acos(torch.dot(vec1, vec2) / (vec1:norm()*vec2:norm()))
end
