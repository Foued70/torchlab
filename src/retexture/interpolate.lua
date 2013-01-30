require 'image'

for f in paths.files(".") do 
   if(string.find(f,"s8.*depth")) then 
      print(f) 
      m = torch.load(f)
      k = 1
      while (m:max() == math.huge) do 
         print("pass: "..k)
         k = k + 1
         for i = 1,m:size(1) do
            for j = 1,m:size(2) do
               if m[i][j] == math.huge then
                  local s = 0
                  local c = 0
                  if i > 1 and m[i-1][j] < math.huge then 
                     s = s + m[i-1][j]
                     c = c + 1
                  end
                  if i < m:size(1) and m[i+1][j] < math.huge then
                     s = s + m[i+1][j]
                     c = c + 1
                  end
                  if j > 1 and m[i][j-1] < math.huge then
                     s = s + m[i][j-1]
                     c = c + 1
                  end
                  if j < m:size(2) and m[i][j+1] < math.huge then
                     s = s + m[i][j+1]
                     c = c + 1
                  end
                  m[i][j] = s/c
               end
            end 
         end
      end
      image.display{image=m,min=0,max=5}
      local fname = f:gsub(".t7","-interp.t7")
      print("Saving: "..fname)
      torch.save(fname,m)
   end
end