require 'torch'
require 'sys'
require 'paths'
require 'math'

local loader = require 'loader'
local Poses = require 'Poses'

local Ray = require 'util.Ray'
local bihtree = require 'util.bihtree'
local interpolate = require 'util.interpolate'
local objloader = require 'util.obj'

local output_dir = paths.concat(paths.dirname(paths.thisfile()), 'cache')
sys.execute("mkdir -p " .. output_dir)

local Occlusions = torch.class('Occlusions')

function Occlusions:__init(posefile, targetfile, scale, packetsize)
  if not posefile or not targetfile then error('arguments invalid') end
  if not paths.filep(posefile) or not paths.filep(targetfile) then 
    error('pose file or target file does not exist') 
  end
    
  self.poses = loader(posefile, Poses.new)
  self.target = loader(targetfile, objloader.new)
  self.scale = scale or 4
  self.packetsize = packetsize
  if packetsize and packetsize < 1 then self.packetsize = nil end
end


function Occlusions:calc()      
  sys.tic()
  local tree = bihtree.build(self.target)
  log.trace("Built tree", sys.toc())
  
  for pi = 1, self.poses.nposes do
    local pose     = self.poses[pi]
    local dirs     = pose:get_dirs(self.scale,packetsize)

    local out_tree = torch.Tensor(dirs:size(1),dirs:size(2))
    local fid_tree = torch.LongTensor(dirs:size(1),dirs:size(2))
    
    sys.tic()
    log.trace("Computing depth map for pose", pi, 'scale 1/', self.scale)

    local tot = 0
    local totmiss = 0
    local pt = pose.xyz
    
    for ri = 1,dirs:size(1) do
      for ci = 1,dirs:size(2) do
        local ray = Ray.new(pt,dirs[ri][ci])
        local tree_d, tree_fid = bihtree.traverse(tree,self.target,ray)
        tot = tot + 1
        out_tree[ri][ci] = tree_d
        fid_tree[ri][ci] = tree_fid
        if (tree_d == math.huge) then
          totmiss = totmiss + 1
        end
      end
    end
    log.trace("Done with pose", pi, sys.toc(), totmiss..'/'..tot)    
    log.trace("Interpolating for "..totmiss.." missed edges")
    
    sys.tic()
    interpolate.math_huge(out_tree)
    log.trace("Interpolation done", sys.toc())
      
    image.display{image={out_tree},min=0,max=5}

    local output_file =  paths.concat(output_dir, pose.name..'_s'..self.scale..'-depth.t7')    
    log.trace("Saving depth map:", output_file)
    torch.save(output_file, out_tree)
   end
   
end


return Occlusions