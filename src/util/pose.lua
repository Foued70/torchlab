require 'sys'
require 'torch'
require 'image'
require 'paths'

pose = {}

function pose.load(posefile)
   sys.tic()
   if (not posefile) then
      posefile = "data/texture_info.txt"
   end
   pasedir  = paths.dirname(posefile)

   local poses = {}
   poses.images = {}
   poses.nposes = 
      tonumber(io.popen(string.format("grep -c '.' %s",posefile)):read())
   print(string.format("Found %d poses in %2.2fs", poses.nposes, sys.toc()))
   sys.tic()
   poses.data = torch.Tensor(poses.nposes,11)

   local pf = io.open(posefile)
   local pc = 1
   for pl in pf:lines() do 
      local n,pd = pl:match("^([%a%d_.]+) (.*)")
      poses[pc] = n
      poses.images[pc] = 
         image.load(string.format("%s/%s", paths.dirname(posefile), n))
      local k = 1
      for n in pd:gmatch("[-.%d]+") do
         poses.data[pc][k] = tonumber(n)
         k = k + 1
      end
      pc = pc + 1
   end
   poses.quat = poses.data:narrow(2,1,4)
   poses.xyz  = poses.data:narrow(2,5,3)
   poses.uv   = poses.data:narrow(2,8,2)
   poses.px   = poses.data:narrow(2,10,2)
   print(string.format("Loaded %d poses in %2.2fs", pc-1, sys.toc()))
   return poses
end

return pose