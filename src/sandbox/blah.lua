z = 3
dq = 3
local dd = torch.zeros(z)
local cnt = 0;
while cnt <= math.pow(2 * dq + 1,z) do
  print(dd)
  local set = 1;
  while set <= z do
    if dd[set] >= 0 then
      if dd[set] < dq then
        dd[set] = -(dd[set] + 1)
        set = z
      else
        dd[set] = 0;
      end

    else
      dd[set] = -dd[set]
      set = z
    end
    set = set + 1
  end
  cnt = cnt + 1
end

print(cnt)