io = require 'io'
imgraph                     = require "../imgraph/init"

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Find Point to Plane matching.')
cmd:text()
cmd:text('Options')
cmd:option('-pointcloud',"arcs/temporary-circle-6132/source/po_scan/a/003/sweep.xyz")
cmd:option('-output_sweep_dir', "output/arcs_temporary-circle-6132_source_po_scan_a_003_sweep_xyz")
cmd:option('-sweep_glob',"saliency_base_9_scale_1.8_n_scale_5_thres_")
cmd:option('-non_interactive',false)
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

_G.pc = PointCloud.PointCloud.new(params.pointcloud)

points    = pc:get_xyz_map()

-- only var map
normals,dd,phi,theta,norm_mask = pc:get_normal_map()


-- set up points and normals
points  = points:reshape(3,points:nElement()/3)
normals = normals:reshape(3,normals:nElement()/3)
local residual_threshold = 100
local normal_threshold   = math.pi/2

printf(" ** scoring with res: %f norm: %f", residual_threshold, normal_threshold)

output_sweep_dir = params.output_sweep_dir
output_glob      =  params.sweep_glob
output_dirs      = util.fs.glob(output_sweep_dir, output_glob)

matcher = Plane.Matcher.new(residual_threshold, normal_threshold)

for _,d in pairs(output_dirs) do 
   collectgarbage()
   print("Processing:", d)
   local planes = torch.load(d.."/planes.t7")

   local score, indices = matcher:match(planes, points, normals)

   score:resize(630,838)
   indices:resize(630,838)
   ic = imgraph.colorize(indices)
   
   for j = 1,3 do 
      ic[j]:cmul(score)
   end

   local score_out = d.."/new_score.jpg"
   print("Saving", score_out)
   image.save(score_out, image.combine(score))

   local planes_out = d.."/new_planes.jpg"
   print("Saving", planes_out)
   image.save(planes_out, image.combine(ic))

   local f = io.open(d.."/new_score.txt", "w")
   f:write(score:mean())
   f:close()
end

-- quit out of luvit if
if params.non_interactive then
   process.exit()
end
