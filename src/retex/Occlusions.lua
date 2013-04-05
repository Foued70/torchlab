-- TODO: instead of posefile and targetfile, pass scan.lua or folder with scan.lua. 

require 'torch'
require 'sys'
require 'paths'
require 'math'

local loader = require 'util.loader'
local Ray = require 'util.Ray'
local bihtree = require 'util.bihtree'
local interpolate = require 'util.interpolate'

local Poses = require('Poses')

local Occlusions = Class()

function Occlusions:__init(posefile, targetfile, scale, packetsize)
  if not posefile or not targetfile then error('arguments invalid') end
  if not paths.filep(posefile) or not paths.filep(targetfile) then 
    error('pose file or target file does not exist') 
  end
    
  self.posefile = posefile
  self.targetfile = targetfile
  self.output_dir = paths.concat(paths.dirname(posefile), 'occlusions')
  sys.execute("mkdir -p " .. self.output_dir)
  
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
        occlusions[i] = torch.load(self:file(i))        
        log.trace('loaded occlusions for pose', i)
      else
        occlusions[i] = nil
        log.trace('no occlusions found for pose', i)
      end
    end
    
    self.occlusions = occlusions
  end
  return self.occlusions
end

function Occlusions:file(pose)
  local occ_file = string.format('%s-%s-p%s-s%s-depth.t7', paths.basename(self.targetfile), 
                    paths.basename(self.posefile), pose, self.scale)
  return paths.concat(self.output_dir, occ_file)
end

function Occlusions:calc()
  local poses = loader(self.posefile, Poses.new)
  local target = loader(self.targetfile, require('util.Obj').new)
  local occlusions = {}
  
  sys.tic()
  local tree = bihtree.build(target)
  log.trace("Built tree in", sys.toc())
  
  for pi = 1, poses.nposes do
    local pose     = poses[pi]
    local dirs     = pose:get_dirs(self.scale,self.packetsize)

    local out_tree = torch.Tensor(dirs:size(1),dirs:size(2))
    local fid_tree = torch.LongTensor(dirs:size(1),dirs:size(2))

    sys.tic()
    log.trace("Computing depth map for pose", pi, 'at scale 1/', self.scale)

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
    log.trace("Depth map done for pose", pi, sys.toc(), totmiss..'/'..tot)    
    log.trace("Interpolating for "..totmiss.." missed edges")

    sys.tic()
    interpolate.math_huge(out_tree)
    log.trace("Interpolation done", sys.toc())
  
    image.display{image={out_tree},min=0,max=5}
    
    local output_file = self:file(pi)
    log.trace("Saving depth map:", output_file)
    torch.save(output_file, out_tree)
    
    occlusions[pi] = out_tree
  end
  
  self.occlusions = occlusions
end