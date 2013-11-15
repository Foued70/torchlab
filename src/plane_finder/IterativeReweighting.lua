local cosine_distance = plane_finder.util.cosine_distance
local residual        = geom.linear_model.residual_fast
local kernel          = util.stats.gaussian_kernel
local score_plane     = plane_finder.util.score_plane_with_normal_threshold
local fit_plane       = geom.linear_model.fit_weighted 
local normal_towards_origin = geom.linear_model.normal_towards_origin

local ss              = nn.SoftShrink(0.1)

-- class iterated reweighted   
local itrw = Class()

function itrw:__init(res_thres, res_decr, res_stop, norm_thres, norm_decr, norm_stop)
   self.res_thres  = res_thres or 100
   self.res_decr   = res_decr or 0.7
   self.res_stop   = res_stop or 1    --  1 mm
   self.norm_thres = norm_thres or math.pi/6
   self.norm_decr  = norm_decr or 0.7
   self.norm_stop  = norm_stop or 0.008 -- 1 degree
   self.n_measurements = 10
   self.save_images = false
   self.image_id    = 0
end

function itrw:score(plane_eqn, points, normals)
   return score_plane(
      plane_eqn , points, normals, self.res_thres,  self.norm_thres, self.n_measurements)
end

function itrw:fit(points, normals, plane_eqn)
   local res_scale    = self.res_thres
   local nrm_scale    = self.norm_thres

   local best_plane   = plane_eqn
   local residuals    = residual(points,best_plane)
   local normal_dists = cosine_distance(normals:t(),best_plane)

   -- score input plane
   local orig_s, orig_c, orig_n_pts = self:score(best_plane, points, normals)

   -- output for gnuplot
   local curves = {{string.format("iteration: %d thres: %2.1f mm norm: %0.3f rad",
                                  0 , res_scale,nrm_scale),orig_c:t()}}
   local best_s     = orig_s
   local best_n_pts = orig_n_pts
   local doresidual = false

   -- TODO other convergence
   for li = 1,128 do 
      collectgarbage()
      -- TODO don't need to recompute all this ... put in get_nbhd function.
      local x = residuals:clone()
      x:div(res_scale)
      x:resize(map_height,map_width)
      local residual_out = kernel(x)
      
      local n = normal_dists:clone()
      n:div(nrm_scale)
      n:resize(map_height,map_width)
      local normal_out = kernel(n)
      
      -- TODO some mixing ? alpha = 0.5
      combo = residual_out:clone()
      combo:cmul(normal_out)
      local combo_out = ss:forward(combo):clone()
      
      mask = combo_out:gt(0)
      npts = mask:sum()

      if npts < 100 then
         printf("only %d points in neighborhood. Stopping",npts)
         break
      else
         filtered_points = torch.Tensor(3,npts)
         for i = 1,3 do 
            filtered_points[i] = points[i][mask] 
         end
         local weights    = combo_out[mask]
         local test_plane = fit_plane(filtered_points, weights)
         normal_towards_origin(test_plane)
         
         local new_s, new_c, new_n_pts = self:score(test_plane, points, normals)
         
         if new_s > best_s then
            printf(" - [%d][%d] score old: %f (%d) new: %f (%d) orig: %f (%d)",
                   self.image_id, li,best_s,best_n_pts,new_s,new_n_pts,orig_s,orig_n_pts)
            table.insert(curves, {string.format("iteration: %d thres: %2.1f mm norm: %0.3f rad",
                                                li, res_scale,nrm_scale),new_c:t()})
            print(" - updating plane")
            best_s = new_s
            best_n_pts = new_n_pts
            best_plane = test_plane
            -- recompute if we change the equation
            -- TODO compute distances function
            residuals    = residual(points,best_plane)
            normal_dists = cosine_distance(normals:t(),best_plane)
            if self.save_images then 
               local inum_str = string.format("%03d_%03d", self.image_id, li)
               local result_str = string.format("s%f_r%2.1f_n%0.3f",new_s,res_scale,nrm_scale)
               -- image.save(string.format("%s_%s_residual.png",    inum,result), image.combine(residual_out))
               -- image.save(string.format("%s_%s_normal.png",      inum,result), image.combine(normal_out))
               image.save(string.format("%s_%s_neighborhood.png",
                                        inum_str,result_str), image.combine(combo_out))
            end
         elseif doresidual and (res_scale > self.res_stop) then 
            res_scale = res_scale * self.res_decr
            printf("Narrowing nbd : res %f mm", res_scale)
            doresidual = not doresidual
         elseif (nrm_scale > self.norm_stop) then 
            nrm_scale = nrm_scale * self.norm_decr
            printf("Narrowing nbd : normal %f radians", nrm_scale)
            doresidual = not doresidual
         else
            break
         end
      end
   end
   return best_plane, best_s, best_n_pts, curves
end
