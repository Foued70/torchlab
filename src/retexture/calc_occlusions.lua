require 'torch'
require 'sys'
require 'paths'
require 'math'

local util = require 'util'

require 'ray'
require 'intersection'
require 'directions'
require 'bihtree'

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
           "models/rivercourt_3307_regeom/rivercourt_3307.obj",
           'target obj with new geometry')
cmd:option('-sourcefile',
           "models/rivercourt_3307_scan/scanner371_job224000.obj",
           'source obj')
cmd:option('-posefile',
           'models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt',
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

sys.tic()
tree = build_tree(target)
printf("Built tree in %2.2fs",sys.toc())

function test_traverse()
   for pi = 1,1 do 
      dirs = load_dirs(poses,pi,scale,packetsize)
      -- dirs = dirs:narrow(1,16,32):narrow(2,112,32)
      out_tree    = torch.Tensor(dirs:size(1),dirs:size(2))
      fid_tree    = torch.LongTensor(dirs:size(1),dirs:size(2))
      out_exhaust = torch.Tensor(dirs:size(1),dirs:size(2))
      fid_exhaust = torch.LongTensor(dirs:size(1),dirs:size(2))
      sys.tic()
      for ri = 1,dirs:size(1) do 
         for ci = 1,dirs:size(2) do 
            local ray = Ray(poses.xyz[pi],dirs[ri][ci])
            
            local tree_d, tree_fid = traverse_tree(tree,target,ray)
            out_tree[ri][ci] = tree_d
            fid_tree[ri][ci] = tree_fid
            
            
            local slow_d,slow_fid = get_occlusions(ray,target)
            if slow_d then
               out_exhaust[ri][ci] = slow_d
               fid_exhaust[ri][ci] = slow_fid
            end
            if (tree_fid ~= slow_fid) or (tree_d ~= slow_d) then
               printf("[%d][%d] tree: %f,%d exhaustive: %f, %d",
                      ri,ci,tree_d,tree_fid, slow_d,slow_fid)
            end
         end
      end
      printf("[%d] Done in %2.2fs",pi,sys.toc())
      image.display{image={out_tree,out_exhaust},min=0,max=5}
   end
end


for pi = 1,poses.nposes do 
misfaces = {}
   dirs = load_dirs(poses,pi,scale,packetsize)
   out_tree    = torch.Tensor(dirs:size(1),dirs:size(2))
   fid_tree    = torch.LongTensor(dirs:size(1),dirs:size(2))
   sys.tic()
   for ri = 1,dirs:size(1) do 
      for ci = 1,dirs:size(2) do 
         local ray = Ray(poses.xyz[pi],dirs[ri][ci])
         
         local tree_d, tree_fid = traverse_tree(tree,target,ray)
        
         if tree_d < 1e6 then
            out_tree[ri][ci] = tree_d
            fid_tree[ri][ci] = tree_fid
         else
           
            local slow_d,slow_fid = get_occlusions(ray,target)
            if slow_d then
               out_tree[ri][ci] = slow_d
               fid_tree[ri][ci] = slow_fid
            end
            if (tree_fid ~= slow_fid) or (tree_d ~= slow_d) then
               if misfaces[slow_fid] then
                  misfaces[slow_fid] = misfaces[slow_fid] + 1
               else
                  misfaces[slow_fid] = 1
               end
            end
         end
      end
   end
   printf("[%d] Done in %2.2fs",pi,sys.toc())
   image.display{image={out_tree},min=0,max=5}
   torch.save(poses[pi]..'_s'..scale..'-depth.t7',out_tree)
   torch.save(poses[pi]..'-misfaces.t7',misfaces)
end

