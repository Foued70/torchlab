-- TODO: instead of posefile and targetfile, pass scan.lua or folder with scan.lua. 

require 'torch'
require 'sys'
require 'paths'
require 'math'

local Poses = require 'Poses'

local loader = require 'util.loader'
local Ray = require 'util.Ray'
local bihtree = require 'util.bihtree'
local interpolate = require 'util.interpolate'
local ObjData = require 'util.obj'

local output_dir = paths.concat(paths.dirname(paths.thisfile()), 'output')
sys.execute("mkdir -p " .. output_dir)

local Occlusions = torch.class('Occlusions')

function Occlusions:__init(posefile, targetfile, scale, packetsize)
  if not posefile or not targetfile then error('arguments invalid') end
  if not paths.filep(posefile) or not paths.filep(targetfile) then 
    error('pose file or target file does not exist') 
  end
    
  self.posefile = posefile
  self.targetfile = targetfile
  self.scale = scale or 4
  self.packetsize = packetsize
  if packetsize and packetsize < 1 then self.packetsize = nil end
end

function Occlusions:get()
  if not self.occlusions then
    local occlusions = {}
    local poses = loader(self.posefile, Poses.new)
    for i=1, poses.nposes do
      if paths.filep(self:file(i)) then 
        self.occlusions[i] = torch.load(self:file(i)) 
      else
        self.occlusions[i] = nil 
      end      
    end
    
    self.occlusions = occlusions
  end
  return self.occlusions
end

function Occlusions:file(pose)
  local occ_file = string.format('%s-%s-p%s-s%s-depth.t7', paths.basename(self.targetfile), 
                    paths.basename(self.posefile), pose, self.scale)
  return paths.concat(output_dir, occ_file)
end

function Occlusions:calc()   
  local poses = loader(self.posefile, Poses.new)
  local target = loader(self.targetfile, ObjData.new)
  
  sys.tic()
  local tree = bihtree.build(target)
  log.trace("Built tree", sys.toc())
  
  for pi = 1, poses.nposes do
    local pose     = poses[pi]
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
        local tree_d, tree_fid = bihtree.traverse(tree,target,ray)
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
    
    local output_file = self:file(pi)
    log.trace("Saving depth map:", output_file)
    torch.save(output_file, out_tree)
  end
   
end


return Occlusions