pi = math.pi
pi2 = pi/2
io               = require 'io'
path             = require 'path'
imgraph          = require "../imgraph/init"
saliency         = require "../image/saliency"
fit_plane        = geom.linear_model.fit
compute_residual = geom.linear_model.residual
plane_finder     = require './plane_finder.lua'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-src_dir','arcs/temporary-circle-6132/source/po_scan/a/001/')
cmd:option('-out_dir','output/')
cmd:option('-thres', 10)
cmd:option('-graph_merge_thres', 5)
cmd:option('-normal_filter', true)
cmd:option('-expanded_points', false)
cmd:option('-add_planes', true)
cmd:option('-use_saliency', false)
cmd:option('-normal_type', 'raw')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)


src_dir = params.src_dir
wrk_dir = src_dir:gsub("source","work")
pcfile = src_dir .. 'sweep.xyz'
rawfile = src_dir .. 'sweep.raw'
out_dir = params.out_dir

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000


threshold             = tonumber(params.thres)
normal_threshold      = math.cos(math.pi/6)
graph_merge_threshold = tonumber(params.graph_merge_thres)

-- saliency
base_win     = 13
scale_factor = 1.4
n_scale      = 7
batch_size   = 1

min_points_for_seed   = 150
min_points_for_plane  = 900 -- 30x30 window is minimum

-- flags
normal_filter   = params.normal_filter
erosion_filter  = false
expanded_points = params.expanded_points
add_planes      = params.add_planes
combine_planes  = false
use_saliency    = params.use_saliency
normal_type     = params.normal_type

_G.planes = {}
_G.error_counts = {}
_G.count = 0

-- setup erosion operator
erosion_amount = 3
dilate_amount  = 0
smooth_amount  = nil
erodeDilate = nil
if dilate_amount > 0 then
   erodeDilate = plane_finder.newErodeDilate(erosion_amount,dilate_amount,smooth_amount,imgh,imgw)
else
   erodeDilate = plane_finder.newErode(erosion_amount,imgh,imgw)
end

-- setup outfile naming
out_dir = out_dir .. "/"..src_dir:gsub("/","_").."/"
if use_saliency then 
   out_dir = string.format("%s/saliency_base_%d_scale_%d_n_scale_%d",
                          out_dir, base_win, scale_factor, n_scale)
else
   out_dir = string.format("%s/segmentation_merge%1.0f", 
                          out_dir, graph_merge_threshold)
end

out_dir = string.format("%s_thres_%d_minseed_%d_minplane_%d",
                       out_dir,
                       threshold, min_points_for_seed, min_points_for_plane)

if normal_filter   then out_dir = out_dir .. string.format("_nf_%1.2f", normal_threshold)  end
if erosion_filter  then out_dir = out_dir .. string.format("_ef_%d", erosion_amount)  end
if expanded_points then out_dir = out_dir .. "_exp" end
if add_planes      then out_dir = out_dir .. "_add" end
if combine_planes  then out_dir = out_dir .. "_cmb" end
out_dir = out_dir .. "_normal_"..normal_type
if not os.execute(string.format("mkdir -p %s", out_dir)) then
   error("error setting up  %s", out_dir)
end

log.tic()
-- load pointcloud
_G.pc     = PointCloud.PointCloud.new(pcfile, max_radius)
xyz_map   = pc:get_xyz_map()
_G.allpts = xyz_map:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()

normals,dd,phi,theta,norm_mask = pc:get_normal_map()
if (normal_type == "var") then 
   normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize() -- smooth_normal()
elseif (normal_type == "var_smooth") then 
   normals,dd,phi,theta,norm_mask = pc:get_normal_map_varsize() -- smooth_normal()
   normals,phi,theta,dd,norm_mask = pc:get_smooth_normal(nil,nil,nil,phi,theta,norm_mask)
elseif (params.normal_type == "smooth") then 
   normals,dd,phi,theta,norm_mask = pc:get_smooth_normal()
end

image.display(normals)

_G.allnrm = normals:reshape(xyz_map:size(1),xyz_map:size(2)*xyz_map:size(3)):t():contiguous()

-- norm_mask == 1 where normals are bad, our mask is where pts are valid
_G.initial_valid = norm_mask:eq(0)
_G.valid = initial_valid:clone()

imgh = normals:size(2)
imgw = normals:size(3)

if use_saliency then

   -- compute all the scales at once and thus reuse the integer image
   _G.salient,_G.sc = saliency.high_entropy_features{img=normals,
                                                     kr=base_win,kc=base_win,
                                                     scalefactor=scale_factor,
                                                     nscale=n_scale}

   patch_mask = valid:clone():zero()

   for scale = n_scale,n_scale-4,-1 do

      -- find patches for initial candidate_planes
      -- whack areas which are masked these should not have low salience.
      _G.inv_salient = sc[scale]:clone()
      max_saliency = inv_salient:max()
      inv_salient:add(-max_saliency):abs()


      -- TODO different x and y window sizes
      final_win = base_win * scale_factor ^ (scale -1)
      _G.nms    = image.NonMaximalSuppression.new(final_win,final_win)
      _G.mx     = nms:forward(inv_salient):squeeze()
      mx:cmul(valid:double()) -- make sure we don't count masked areas as non-salient

      -- TODO fix this bug around the far edge
      bmx_useable_h = imgh-final_win
      bmx_useable_w = imgw-final_win
      n_patches = math.huge
      while (mx:max() > 0) and (n_patches > 0) do
         _G.candidate_planes = {}
         printf("max saliency = %f", mx:max())
         _G.bmx    = mx:gt(0)

         idx,val,n_patches = plane_finder.get_vals_index(bmx[{{1,bmx_useable_h},{1,bmx_useable_w}}])

         printf("found %d patches",n_patches)
         if (n_patches > 0) then
            s,si = torch.sort(val)

            tot_win_pts = final_win * final_win
            -- loop through patches validating candidate_planes
            -- when n_patches > batch_size only search in top 1/2 patches
            local n_patch   = math.min(n_patches-1,
                                       math.max(batch_size,math.floor(n_patches*0.5-1)))
            local step_size = math.max(1,math.floor(n_patch/batch_size)) -- max batch_size patches searched
            printf("processing batch of %d",(n_patch/step_size))
            for ii = n_patches,n_patches-n_patch,-step_size do
               i = si[ii]
               -- remove this value whether accepted or not
               bbx = plane_finder.compute_bbx(idx[{{},i}],final_win,final_win,imgh,imgw)
               -- clear mx so we don't check the same patch twice
               mx[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 0
               patch_mask:fill(0)
               patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1
               patch_mask:cmul(valid)

               current_plane, error_string =
                  plane_finder.from_seed{
                     points               = allpts,
                     mask                 = patch_mask,
                     threshold            = threshold,
                     min_points_for_seed  = min_points_for_seed,
                     min_points_for_plane = min_points_for_plane,
                     expanded_set         = explanded_set,
                     normal_filter        = normal_filter,
                     normals              = allnrm,
                     normal_threshold     = normal_threshold,
                     erosion_filter       = erosion_filter,
                     erosion_function     = erodeDilate,
                     debug = {{
                                 win_idx  = idx[{{},i}]:clone(),
                                 win_size = {x=final_win, y=final_win}
                              }}
                  }
               -- TODO the add and combine processing is the same for segm and saliency should be a function.
               printf(" - error: %s", error_string)

               if error_counts[error_string] then
                  error_counts[error_string] = error_counts[error_string] + 1
               else
                  error_counts[error_string] = 1
               end

               if (not planes) or (#planes == 0) and current_plane then
                  _G.planes = {current_plane}
               elseif add_planes then
                  _G.planes = plane_finder.add_planes(allpts, planes, {current_plane}, threshold, erodeDilate)
               else
                  table.insert(planes, current_plane)
               end
               printf(" - Total %d planes in %d patches", #planes, count)
               count = count + 1

               -- combine
               n_planes = #planes
               if combine_planes and (n_planes > 0) and (n_planes % 10 == 0) then
                  _G.planes = plane_finder.combine_planes(allpts, planes, threshold, erodeDilate)
                  changed = n_planes - #planes
                  if (changed > 0) then
                     printf(" - Combined %d pairs of %d leaving %d", changed, n_planes, #planes)
                  end
               end

               collectgarbage()

            end -- for
         end -- if
      end -- while still valid in this scale
      image.save(string.format("%s/scale_%dx%d.png",out_dir,final_win,final_win),
                 plane_finder.visualize_planes(planes))
   end -- for scales
else

   -- make segm patches
   _G.data = torch.Tensor(3,phi:size(1),phi:size(2))

   data[1]:copy(phi):add(-phi:min()):mul(1/(phi:max()-phi:min()))
   data[2]:copy(theta):add(-theta:min()):mul(1/(theta:max()-theta:min()))
   d = data[3]
   d:copy(torch.cmul(xyz_map,normals):resize(3,xyz_map[1]:nElement()):sum(1):squeeze())
   d:add(1):log()
   d:mul(1/d:max())

   datag = image.convolve(data, image.gaussian(7), 'same')
   graph = imgraph.graph(datag)
   mstsegm = imgraph.segmentmst(graph, graph_merge_threshold, min_points_for_seed)

   graph_rgb = image.display(imgraph.colorize(mstsegm)):clone()

   h = {}
   mstsegm:apply(function (x)
                    if h[x] then
                       h[x] = h[x] + 1
                    else
                       count = count + 1
                       h[x] = 1
                    end
                 end)

   printf("Found %d segments",count)
   _G.v = torch.LongTensor(2,count)
   count = 1
   for id,quant in pairs(h) do
      v[1][count] = quant
      v[2][count] = id
      count = count+1
   end

   s, si = torch.sort(v[1])

   for patch_count = count-1,1,-1 do
      ii = si[patch_count]
      segm_count = v[1][ii]
      segm_id    = v[2][ii]

      printf(" - processing segm id: %d with %d pixels", segm_id, segm_count)

      segm_mask = mstsegm:eq(segm_id):cmul(valid)

      current_plane, error_string =
         plane_finder.from_seed{
            points               = allpts,
            mask                 = segm_mask,
            threshold            = threshold,
            min_points_for_seed  = min_points_for_seed,
            min_points_for_plane = min_points_for_plane,
            expanded_set         = explanded_set,
            normal_filter        = normal_filter,
            normals              = allnrm,
            normal_threshold     = normal_threshold,
            erosion_filter       = erosion_filter,
            erosion_function     = erodeDilate,
            debug = {{
                        segm_ids = segm_id,
                        segm_counts = segm_count
                     }}
         }

      printf(" - error: %s", error_string)

      if error_counts[error_string] then
         error_counts[error_string] = error_counts[error_string] + 1
      else
         error_counts[error_string] = 1
      end

      if (not planes) or (#planes == 0) and current_plane then
         _G.planes = {current_plane}
      elseif add_planes then
         _G.planes = plane_finder.add_planes(allpts, planes, {current_plane}, threshold, erodeDilate)
      else
         table.insert(planes, current_plane)
      end
      printf(" - Total %d planes so far in %d/%d", #planes, count-patch_count + 1, count)

      -- combine
      n_planes = #planes
      if combine_planes and (n_planes > 0) and (n_planes % 10 == 0) then
         _G.planes = plane_finder.combine_planes(allpts, planes, threshold, erodeDilate)
         changed = n_planes - #planes
         if (changed > 0) then
            printf(" - Combined %d pairs of %d leaving %d", changed, n_planes, #planes)
         end
      end
   end

   collectgarbage()

end

toc = log.toc()

-- score
pstd = torch.Tensor(#planes)

for i,p in pairs(planes) do
   valid[p.mask] = 0
   pstd[i]       = p.score
end
found_pts = valid:sum()
total_pts = initial_valid:sum()
percent_found = 100 * found_pts/total_pts

rgb = plane_finder.visualize_planes(planes)

_G.outname = out_dir .. "/planes"
image.save(outname .. ".png", rgb)
if not use_saliency then
   image.save(out_dir  .. "/segmentation.png", graph_rgb)
end
image.save(out_dir  .. "/normals.png", image.combine(normals))
image.save(out_dir  .. "/valid.png", valid:mul(255))
torch.save(outname .. ".t7", planes)
-- write score
score_fname = outname .. "-score.txt"
score_file = io.open(score_fname,"w")
score_file:write("Errors:\n")
for str,cnt in pairs(error_counts) do
   score_file:write(string.format("%s %d\n",str,cnt))
end
score_file:write("Scores:\n")
score_file:write(string.format("patches searched %d\n",count))
score_file:write(string.format("planes found %d\n",#planes))
score_file:write(string.format("score mean %f min %f max %f \n",pstd:mean(),pstd:min(), pstd:max()))
score_file:write(string.format("points explained %d/%d %2.1f%%\n",found_pts, total_pts, percent_found))
score_file:write(string.format("time: %fs\n", toc*1e-3))
score_file:close()
os.execute("cat "..score_fname)

-- save obj
quat, aapts = plane_finder.align_planes(planes,allpts)
plane_finder.aligned_planes_to_obj (planes, aapts, quat, outname..".obj")
