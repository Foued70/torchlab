io = require 'io'
imgraph          = require "../imgraph/init"
graph_merge_threshold = 5
min_points_for_seed = 900

planefile = "arcs/motor-unicorn-0776/work/planes/sweep_001/saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var/planes.t7"

pcfile = "arcs/motor-unicorn-0776/source/faro/sweep_001.xyz"

_G.d = Plane.CollectData.new(pcfile, planefile)

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
   

function write_bbox(mask,xyz_map,pi,si)
   local pxh = mask:size(1)
   local pxw = mask:size(2)
   local b = {}

   local pts  = mask_to_points(mask)
   pts = pts:t():contiguous()
   print(pts)
   local hull = opencv.imgproc.convexHull(opencv.Mat.new(pts))
   hull = hull:toTensor():squeeze()
   for i = 1,hull:size(1) do 
      pt = hull[i]
      table.insert(b,xyz_map[{pt[1],pt[2],{}}])
   end
   
   ptsf = io.open(string.format('points_%03d_%03d.xyz',pi,si), 'w')

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

   objf = io.open(string.format('face_%03d_%03d.obj',pi,si), 'w')
   for _,p in pairs(b) do 
      p = p:squeeze()
      objf:write(string.format("v %d %d %d\n",p[1],p[2],p[3]))
   end
   objf:write("\n\n")

   -- write faces
   local s = "f "
   
   for i = 1,hull:size(1) do
      s = string.format("%s %d",s,i)
   end
   objf:write(s.."\n")
   objf:close()
   return b
end


for pi = 1,#d.eqns do 
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
   for idx,n_pts in pairs(h) do
      _G.mask    = mstsegm:eq(idx)
      occ_tot = occupied[mask]:sum()
      p_occ = occ_tot/n_pts
      if p_occ > 0.9 then
         out_mask[mask] = 1 
         write_bbox(mask,xyz_map,pi,si)
         si = si + 1
      end
      printf("plane %d percent occupied: %f n pts: %d", idx, p_occ, n_pts)
   end

   for i = 1,3 do
      graph_rgb[i]:cmul(out_mask)
   end

   out_name = string.format("occupied_segments_%03d.jpg", pi)
   printf("saving: %s",out_name)
   image.save(out_name, graph_rgb)

end


