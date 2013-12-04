io = require 'io'
os = require 'os'
imgraph          = require "../imgraph/init"
graph_merge_threshold = 5
min_points_for_seed = 900

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Create obj from plane segments')
cmd:text()
cmd:text('Options')

cmd:option('-planefile', "arcs/motor-unicorn-0776/work/planes/sweep_001/saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var/planes.t7")
cmd:option('-pcfile',"arcs/motor-unicorn-0776/source/faro/sweep_001.xyz")
cmd:option('-outdir',"output/")
cmd:option('-uvs',false)
cmd:text()

-- parse input params
params      = cmd:parse(process.argv)
pcfile      = params.pcfile
planefile   = params.planefile
outdir      = params.outdir
compute_uvs = params.uvs

print("Making ".. outdir)
if not os.execute(string.format("mkdir -p %s", outdir)) then
   error("error setting up  %s", outdir)
end

_G.d = Plane.CollectData.new(pcfile, planefile)
print("OK built collection")
collectgarbage()

_G.bbx = {}

function mask_to_points(mask)
   local pxh = mask:size(1)
   local pxw = mask:size(2)
   local n_pts = mask:sum()
   local index = torch.FloatTensor(2,pxh,pxw)
   index[1]:copy(torch.linspace(1,pxh,pxh):resize(pxh,1):expandAs(mask))
   index[2]:copy(torch.linspace(1,pxw,pxw):resize(1,pxw):expandAs(mask))
   local pts = index[mask:reshape(1,pxh,pxw):expandAs(index)]
   pts:resize(2,n_pts)
   return pts
end 

-- TODO should get angles of grid, xyz to angle and angle to xyz
-- this hack finds grid position of closest point and uses that to calculate uv.
function get_uvs(pts, xyz_map, mask)
   local h = xyz_map:size(2)
   local w = xyz_map:size(3)
   local xyz = xyz_map:reshape(3,h*w)
   local uvs = {}
   for _,pt in pairs(pts) do 
      d   = Plane.util.get_distance(pt,xyz)
      m,i = d:min(1)
      idx = i[1]
      y   = math.floor(idx/w)
      x   = idx - (y*w)
      table.insert(uvs,{y/h,x/w})
   end
   return uvs
end

function get_hull(mask,xyz_map)
   local b = {}

   local pts  = mask_to_points(mask)
   pts = pts:t():contiguous()
   local hull = opencv.imgproc.convexHull(opencv.Mat.new(pts))
   hull = hull:toTensor():squeeze()
   for i = 1,hull:size(1) do 
      pt = hull[i]
      table.insert(b,xyz_map[{pt[1],pt[2],{}}])
   end
   return b
end

function write_pts(xyz_map,mask,pi,si)
   -- write pts.xyz
   ptsf = io.open(string.format('%s/points_%03d_%03d.xyz',outdir,pi,si), 'w')

   mask:resize(mask:nElement())
   oxyz = xyz_map:reshape(pxh*pxw,3)
   validxyz = oxyz[mask:resize(mask:size(1),1):expandAs(oxyz)]
   validxyz:resize(validxyz:nElement()/3,3)

   for i = 1,validxyz:size(1),10 do 
      collectgarbage()
      pt = validxyz[i]
      if torch.abs(pt):sum() > 0 then
         ptsf:write(string.format("%f %f %f\n", pt[1], pt[2], pt[3]))
      end
   end
   ptsf:close()
end

function write_obj(b,uvs,pi,si)
   local pxh = mask:size(1)
   local pxw = mask:size(2)


   -- write vertices
   objf = io.open(string.format('%s/face_%03d_%03d.obj',outdir,pi,si), 'w')
   for pi,p in pairs(b) do 
      p = p:squeeze()
      objf:write(string.format("v %d %d %d\n",p[1],p[2],p[3]))
   end
   objf:write("\n\n")

   -- write texture coords and faces
   local s = "f "
   if uvs then
      for _,uv in pairs(uvs) do 
         objf:write(string.format("vt %f %f \n",uv[1],uv[2]))
      end
      objf:write("\n\n")
      
      for i = 1,#b do
         s = string.format("%s %d/%d",s,i,i)
      end
   else
      for i = 1,#b do
         s = string.format("%s %d",s,i)
      end
   end
   objf:write(s.."\n")
   objf:close()
   return b
end

_, pc_mask = d.pc:get_index_and_mask()
pc_xyz_map = d.pc:get_xyz_map()

for pi = 1,#d.eqns do 
   collectgarbage()
   print("Processing "..pi)
   if pi>1 then 
      d:update_plane(pi)
   end
   local occupied = d.ioccupiedF:double()
   local pxh      = occupied:size(1)
   local pxw      = occupied:size(2)
   local xyzND    = d:get_xyz_coordinates()
   local xyz      = xyzND:t():contiguous() 
      
   local eqn = d:getPlaneEquation()
   
   local _,idx = torch.abs(eqn[{{1,3}}]):max(1)
   
   idx = idx[1]

   datag     = image.convolve(occupied, image.gaussian(graph_merge_threshold), 'same')
   graph     = imgraph.graph(datag)
   mstsegm   = imgraph.segmentmst(graph, graph_merge_threshold, min_points_for_seed)
   graph_rgb = image.combine(imgraph.colorize(mstsegm))

   h = {}
   n_segm = 0
   mstsegm:apply(function (x)
                    if h[x] then
                       h[x] = h[x] + 1
                    else
                       n_segm = n_segm + 1
                       h[x] = 1
                    end
                 end)

   printf("Found %d segments",n_segm)

   xyz_map = xyzND:reshape(pxh,pxw,3)

   out_mask = torch.zeros(occupied:size())
   si = 1
   uvs = nil
   for idx,n_pts in pairs(h) do
      _G.mask    = mstsegm:eq(idx)
      occ_tot = occupied[mask]:sum()
      p_occ = occ_tot/n_pts
      if p_occ > 0.9 then
         out_mask[mask] = 1
         b = get_hull(mask,xyz_map)
         if compute_uvs then 
            uvs = get_uvs(b,pc_xyz_map,pc_mask)
         end
         write_obj(b,uvs,pi,si)
         si = si + 1
      end
      printf("plane %d percent occupied: %f n pts: %d", idx, p_occ, n_pts)
   end

   for i = 1,3 do
      graph_rgb[i]:cmul(out_mask)
   end

   out_name = string.format("%s/occupied_segments_%03d.jpg", outdir, pi)
   printf("saving: %s",out_name)
   image.save(out_name, graph_rgb)

end


