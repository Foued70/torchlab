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

-- input:  Nx3 tensor of unit cartesian vectors
-- output: Nx2 tensor of azimuth and elevation for a set of points
-- these are in camera coords
--    x : right
--    y : forward
--    z : up
function unit_cartesian_to_spherical_angles(uc)

   local x = uc[{{},1}]
   local y = uc[{{},2}]
   local z = uc[{{},3}]

   local angles = torch.Tensor(uc:size(1),2)

   angles[{{},1}] = torch.atan2(x,y) -- azimuth
   angles[{{},2}] = torch.asin(z)   -- elevation

   return angles

end

-- input:  Nx2 tensor of azimuth and elevation for a set of points
-- output: Nx3 tensor of unit cartesian vectors
-- these are in camera coords
--    x : right
--    y : forward
--    z : up
function spherical_angles_to_unit_cartesian(angles)

   local azimuth   = angles[{{},1}]
   local elevation = angles[{{},2}]

   local unit_vec = torch.Tensor(angles:size(1),3)
   local cos_elevation = torch.cos(elevation)

   -- right = sin(azimuth) * cos(elevation)
   unit_vec[{{},1}]  = torch.sin(azimuth):cmul(cos_elevation)
   -- forward = cos(azimuth) * cos(elevation)
   unit_vec[{{},2}]  = torch.cos(azimuth):cmul(cos_elevation)
   -- up = sin(elevation)  or cos(pi/2 - elevation) == cos(polar)
   unit_vec[{{},3}]  = torch.sin(elevation)

   return unit_vec
end

