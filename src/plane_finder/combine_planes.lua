pi = math.pi
pi2 = pi/2
io               = require 'io'
path             = require 'path'
imgraph          = require "../imgraph/init"
saliency         = require "../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual
plane_combiner     = require './plane_combiner.lua'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-src_file','arcs/temporary-circle-6132/source/po_scan/a/001/sweep.xyz')
cmd:option('-planes_file','')
cmd:option('-out_dir','output/')
cmd:option('-thres', 10)
cmd:option('-min_pts_for_seed', 150)  -- < 20 x 20 ...
cmd:option('-min_pts_for_plane', 900) -- 30x30 window is minimum
cmd:option('-normal_filter', true)
cmd:option('-normal_type', 'raw')
cmd:option('-non_interactive',false)
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

src_file = params.src_file
out_dir  = params.out_dir

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000

threshold             = tonumber(params.thres)
normal_threshold      = math.cos(math.pi/6)
graph_merge_threshold = tonumber(params.graph_merge_thres)


-- TODO add these to args
min_points_for_seed   = tonumber(params.min_pts_for_seed)
min_points_for_plane  = tonumber(params.min_pts_for_plane)

-- flags
normal_filter     = params.normal_filter
normal_type       = params.normal_type

planes_file = params.planes_file
if util.fs.is_file(planes_file) then
   tmp = torch.load(planes_file)
   _G.planes = {}
   -- choose planes larger than min_points_for_plane
   for _,p in pairs(tmp) do 
      if p.n_pts > min_points_for_plane then 
         p.debug_info[1].id = #planes+1
         table.insert(planes,p)
      end
   end
else
   error("can't open "..planes_file)
end

_G.combine_scores = {}
_G.loser_scores = {}
_G.count    = 1
last_tested = 1

-- setup outfile naming
out_dir = out_dir .. "/".. src_file:gsub("[/%.]","_") .."/"
out_dir = string.format("%s/combination_", out_dir)

out_dir = string.format("%s_thres_%d_minseed_%d_minplane_%d",
                        out_dir,
                        threshold, min_points_for_seed, min_points_for_plane)

if normal_filter   then out_dir = out_dir .. string.format("_nf_%1.2f", normal_threshold)  end
out_dir = out_dir .. "_normal_"..normal_type
if not os.execute(string.format("mkdir -p %s", out_dir)) then
   error("error setting up  %s", out_dir)
end

file_bname = path.basename(out_dir)
log.tic()
-- load pointcloud
_G.pc     = PointCloud.PointCloud.new(src_file, max_radius)
xyz_map   = pc:get_xyz_map()
map_height = xyz_map:size(2)
map_width  = xyz_map:size(3)
allpts     = xyz_map:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()
_G.test_points = allpts

normals,dd,phi,theta,norm_mask = pc:get_normal_map()
if (normal_type == "var") then
   normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
elseif (normal_type == "var_smooth") then
   normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize()
   normals,phi,theta,dd,norm_mask = pc:get_smooth_normal(nil,nil,nil,phi,theta,norm_mask)
elseif (normal_type == "smooth") then
   normals,dd,phi,theta,norm_mask = pc:get_smooth_normal()
end


allnrm = torch.Tensor(3,xyz_map:size(2)*xyz_map:size(3))
allnrm[{{1,3},{}}]:copy(normals)
-- allnrm[{4,{}}]:copy(dd)
allnrm = allnrm:t():contiguous()
_G.test_nrm = allnrm
collectgarbage()

-- norm_mask == 1 where normals are bad, our mask is where pts are valid
_G.initial_valid = norm_mask:eq(0)
_G.valid = initial_valid:clone()


-- combine
n_planes = #planes
_G.combine_scores1 = {}
_G.loser_scores1 = {}

if (n_planes > 0) then
   combined_planes = plane_combiner.combine_planes(allpts, planes, threshold,
                                                   normal_filter, normal_threshold, allnrm,
                                                   combine_scores, loser_scores)
end
_G.test_combined_planes = combined_planes

n_combined_planes = #combined_planes

toc = log.toc()

-- score
pstd = torch.Tensor(#combined_planes)

for i,p in pairs(combined_planes) do
   valid[p.mask] = 0
   pstd[i]       = p.score
end

remain_pts     = valid:sum()
total_pts      = initial_valid:sum()
percent_remain = 100 * remain_pts/total_pts
found_pts      = total_pts - remain_pts
percent_found  = 100 - percent_remain

if (n_combined_planes > 0) then
   rgb = plane_combiner.visualize_planes(combined_planes, map_height, map_width)
end

_G.outname = out_dir .. "/planes"
image.save(outname .. ".png", rgb)
image.save(out_dir  .. "/normals.png", image.combine(normals))
image.save(out_dir  .. "/valid.png", valid:mul(255))

-- write score
score_fname = outname .. "-score.txt"
score_file = io.open(score_fname,"w")
score_file:write("Scores:\n")
score_file:write(string.format("planes combined %d from %d\n",#combined_planes, #planes))
score_file:write(string.format("score mean %f min %f max %f \n",pstd:mean(),pstd:min(), pstd:max()))
score_file:write(string.format("points explained %d/%d %2.1f%%\n",found_pts, total_pts, percent_found))
score_file:write(string.format("time: %fs\n", toc*1e-3))
score_file:write("Combine Scores:\n")
for str,tbl in pairs(combine_scores) do
   score_file:write(string.format("%s %d\n", str, tbl.cnt))
end
score_file:write("Loser Scores:\n")
for str,tbl in pairs(loser_scores) do
   score_file:write(string.format("%s %d\n", str, tbl.cnt))
end

score_file:close()
os.execute("cat "..score_fname)

-- write scoreline
score_fname = outname .. "-scoreline.txt"
score_file = io.open(score_fname,"w")
score_file:write("# ")
score_file:write("norm_1=raw,2=smooth,3=var,4=var_smooth normal_threshold ")
score_file:write("minseed minplane ")
score_file:write("pfound scmean scmin scmax found_pts remain_pts total_pts percent_found time\n")
score_file:write(string.format("%s ", file_bname))
score_file:write(string.format("%d %f ",
                               normal_type == "raw" and 1 or
                                  (normal_type == "smooth" and 2) or
                                  (normal_type == "var" and 3) or
                                  (normal_type == "var_smooth" and 4) or 0,
                               normal_threshold))
score_file:write(string.format("%d %d ", min_points_for_seed, min_points_for_plane))
score_file:write(string.format("%d ",#planes))
score_file:write(string.format("%f %f %f ",pstd:mean(),pstd:min(), pstd:max()))
score_file:write(string.format("%d %d %d %2.1f ",found_pts, remain_pts, total_pts, percent_found))
score_file:write(string.format("%f\n", toc*1e-3))
score_file:close()

os.execute("cat "..score_fname)

-- save obj
quat, aapts = plane_combiner.align_planes(combined_planes,allpts)
plane_combiner.aligned_planes_to_obj (combined_planes, aapts, quat, outname)

-- save planes with out masks
for _,p in pairs(combined_planes) do
   p.mask = nil
end
torch.save(outname .. ".t7", combined_planes)

-- quit out of luvit if
if params.non_interactive then
   process.exit()
end
