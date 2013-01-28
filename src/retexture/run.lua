require 'torch'
require 'sys'
require 'paths'
require 'math'
local util = require 'util'

require 'ray'
require 'intersection'
require 'directions'
require 'bihtree'

local geom = util.geom
-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
-- cmd:option('-targetfile',
--           "rivercourt_3307_scan/rivercourt_3307.obj",
--           'target obj with new geometry')

cmd:option('-targetfile',
           "/Users/marco/lofty/test/invincible-violet/retexture-tworoom.obj",
           'target obj with new geometry')
cmd:option('-sourcefile',
           "/Users/marco/lofty/models//invincible-violet-3396_a_00/scanner371_job129001.obj",
           'source obj')
cmd:option('-posefile',
           "/Users/marco/lofty/models//invincible-violet-3396_a_00/scanner371_job129001_texture_info.txt",
           'pose info file in same directory as the texture images')
cmd:option('-scale',1,'scale at which to process 4 = 1/4 resolution')
cmd:option('-packetsize',0,'window size for ray packets (32x32)')
cmd:text()
 
-- parse input params
params = cmd:parse(arg)

targetfile = params.targetfile
sourcefile = params.sourcefile
posefile   = params.posefile
scale      = params.scale
packetsize = params.packetsize 
if packetsize < 1 then packetsize = nil end
cachedir = "cache/"
sys.execute("mkdir -p " .. cachedir)

posecache   = cachedir .. posefile:gsub("/","_")   .. ".t7"
sourcecache = cachedir .. sourcefile:gsub("/","_") .. ".t7"
targetcache = cachedir .. targetfile:gsub("/","_") .. ".t7"
posedir = paths.dirname(posefile)

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
   poses  = loadcache(posefile,posecache,util.pose.loadtxtfile)
end
if not source then
   source = loadcache(sourcefile,sourcecache,util.obj.load,3)
end
if not target then
   target = loadcache(targetfile,targetcache,util.obj.load,4)
end


tree = build_tree(target)

pi = 1 
dirs = load_dirs(poses,pi,scale,packetsize)

print(dirs:size())

out_tree    = torch.Tensor(dirs:size(1),dirs:size(2))
out_exhaust = torch.Tensor(dirs:size(1),dirs:size(2))

sys.tic()

for ri = 1,dirs:size(1) do 
   for ci = 1,dirs:size(2) do 
      local ray = Ray(poses.xyz[pi],dirs[ri][ci])
      out_tree[ri][ci] = traverse_tree(tree,target,ray)
   
      -- local slow_d,slow_fid = get_occlusions(ray.origin,ray.dir,target)
      -- out_exhaust[ri][ci] = slow_d
      -- printf("tree: %f exhaustive: %f, %d",out_tree[ri][ci],slow_d,slow_fid)
   end
end
printf("Done in %2.2fs",sys.toc())
