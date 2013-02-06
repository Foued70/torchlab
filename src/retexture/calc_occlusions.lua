require 'torch'
require 'sys'
require 'paths'
require 'math'

local util = require 'util'

local Ray = util.Ray

require 'interpolate'
require 'bihtree'

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()

cmd:option('-targetfile',
           "models/rivercourt_3307_regeom/rivercourt_3307.obj",
           --'test/invincible-violet/retexture-tworoom.obj',
           'target obj with new geometry')
cmd:option('-sourcefile',
           "models/rivercourt_3307_scan/scanner371_job224000.obj",
           -- 'models/invincible-violet-3396_a_00/scan.obj',
           'source obj')
cmd:option('-posefile',
           'models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt',
           -- 'models/invincible-violet-3396_a_00/scanner371_job129001_texture_info.txt',
           'pose info file in same directory as the texture images')
cmd:option('-outdir','output/')
cmd:option('-scale',4,'scale at which to process 4 = 1/4 resolution')
cmd:option('-packetsize',0,'window size for ray packets (32x32)')
cmd:text()

-- parse input params
local params = cmd:parse(arg)

local targetfile = params.targetfile
local sourcefile = params.sourcefile
local posefile   = params.posefile
local outdir     = params.outdir .. "/"
local scale      = params.scale
local packetsize = params.packetsize

if packetsize < 1 then packetsize = nil end
local cachedir = "cache/"
sys.execute("mkdir -p " .. cachedir)
sys.execute("mkdir -p " .. outdir)

local posecache   = cachedir .. posefile:gsub("/","_")   .. ".t7"
local sourcecache = cachedir .. sourcefile:gsub("/","_") .. ".t7"
local targetcache = cachedir .. targetfile:gsub("/","_") .. ".t7"
local posedir = paths.dirname(posefile)

function loadcache (objfile,cachefile,loader,args)
   local object = nil
   -- Process or load the poses
   if (paths.filep(cachefile)) then
      sys.tic()
      object = torch.load(cachefile)
      printf("Loaded %s from %s in %2.2fs", objfile, cachefile, sys.toc())
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      printf("Saving %s to %s", objfile, cachefile)
   end
   return object
end

if not poses then
   poses  = loadcache(posefile,posecache,Poses.new)
   poses.cachedir = posecache
end
-- if not source then
--    source = loadcache(sourcefile,sourcecache,util.obj.load,3)
-- end
if not target then
   target = loadcache(targetfile,targetcache,util.obj.load,4)
end

sys.tic()
tree = build_tree(target)
printf("Built tree in %2.2fs",sys.toc())

function do_calc_occlusions(save)
   for pi = 1,poses.nposes do
      local pose     = poses[pi]
      local dirs     = pose:get_dirs(scale,packetsize)

      local out_tree = torch.Tensor(dirs:size(1),dirs:size(2))
      local fid_tree = torch.LongTensor(dirs:size(1),dirs:size(2))
      sys.tic()
      printf("Computing depth map for pose[%d] at 1/%d scale",pi,scale)
      local tot = 0
      local totmiss = 0
      local pt = pose.xyz
      for ri = 1,dirs:size(1) do
         for ci = 1,dirs:size(2) do
            local ray = Ray.new(pt,dirs[ri][ci])

            local tree_d, tree_fid = traverse_tree(tree,target,ray)
            tot = tot + 1
            out_tree[ri][ci] = tree_d
            fid_tree[ri][ci] = tree_fid
            if (tree_d == math.huge) then
               totmiss = totmiss + 1
            end
         end
      end
      printf("[%d] Done in %2.2fs %d/%d",pi,sys.toc(),totmiss,tot)
      printf("interpolating for %d missed edges", totmiss)
      sys.tic()
      interpolate(out_tree)
      printf(" - done in %2.2fs",sys.toc())
      
      image.display{image={out_tree},min=0,max=5}

      if save then
         local outdfname = outdir..pose.name..'_s'..scale..'-depth.t7'
         printf("Saving: %s", outdfname)
         torch.save(outdfname,out_tree)
         torch.save(outdir..pose.name..'-misfaces.t7',misfaces)
      end
   end
end

do_calc_occlusions(true)
