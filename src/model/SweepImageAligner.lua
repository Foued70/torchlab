path = require 'path'
blend = projection.util.blend
optimize = util.optimize.convex_binary_search
pi  = math.pi
pi2 = pi * 0.5

SweepImageAligner = Class()

function SweepImageAligner:__init(imagePathOrData)
 
   -- imagePathOrData can be a string 
   if type(imagePathOrData) == "string" then
      self.image_filenames = 
         string.split(util.fs.exec(string.format('ls %s',imagePathOrData)))
   elseif type(imagePathOrData) == "table" then
      if type(imagePathOrData[1]) == "string" then
         -- table of image filenames
         self.image_filenames = imagePathOrData
      elseif type(imagePathOrData[1]) == "userdata" then
         -- table of preloaded images
         self.images = imagePathOrData
      else
         error("don't understand type of elements in table")
      end
   else
      error("don't understand type of arguement")
   end

   -- ** Input images **
   n_images = 0
   if self.images then 
      n_images = #self.images
   else
      n_images = #self.image_filenames
   end
   printf("Found %d images",n_images)
   -- images are vertical
   vfov         = 1.5290 
   hfov         = (74.8/180) * pi

   scale        = 0.2
   store_images = false
   image_wand   = image.Wand.new()

   -- ** Projections **
   force        = true

   lambda_guess       = 2*pi/n_images
   phi_guess          = 0
   lambda_wiggle_base = hfov / 40 -- in radians 
   phi_wiggle_base    = vfov / 40 -- in radians 
   vfov_wiggle_base   = phi_wiggle_base
   hfov_wiggle_base   = lambda_wiggle_base
   
   -- ** Scores
   -- setup best first guess
   delta = torch.Tensor(2,n_images)
   delta[1]:fill(lambda_guess)
   delta[2]:fill(phi_guess)

   scores = torch.Tensor(n_images);

   width, height = self:get_input_width_height()
 
   -- class variables
   self.n_images           = n_images
   self.input_vfov         = vfov
   self.input_hfov         = hfov

   self.scale              = scale
   self.store_images       = store_images
   self.image_wand         = image_wand
   self.force              = force

   self.lambda_guess       = lambda_guess
   self.phi_guess          = phi_guess
   self.lambda_wiggle_base = lambda_wiggle_base
   self.phi_wiggle_base    = phi_wiggle_base
   self.vfov_wiggle_base   = vfov_wiggle_base
   self.hfov_wiggle_base   = hfov_wiggle_base
   
   self.delta              = delta
   self.scores             = scores

end

function SweepImageAligner:get_input_projection(force)
   if (not self.input_projection) or force then 
      width,height = self:get_input_width_height()
      hfov = self.input_hfov
      hfov = self.input_hfov
      self.input_projection  = 
         projection.GnomonicProjection.new(width,height,hfov,vfov)
   end
   return self.input_projection
end

-- pair is the workspace for a pair of images
function SweepImageAligner:get_pair_projection(force)
   if (not self.pair_projection) or force then 
      width, height   = self:get_input_width_height()
      pair_hfov       = self.input_hfov + self.lambda_guess + self.lambda_wiggle_base
      pair_vfov       = self.input_vfov + self.phi_guess + 2 * self.phi_wiggle_base
      pair_width      = width  * pair_hfov/self.input_hfov
      pair_height     = height * pair_vfov/self.input_vfov
      self.pair_projection = 
         projection.SphericalProjection.new(pair_width,pair_height,pair_hfov,pair_vfov)
   end
   return self.pair_projection
end

-- output full equirectangular
function SweepImageAligner:get_output_projection(force)
   if (not self.output_projection) or force then
      width, height = self:get_input_width_height()
      output_hfov   = 2 * pi
      output_vfov   = self.input_vfov + 0.1 * self.input_vfov
      output_height = height + 0.1 * height
      output_width  = output_height * output_hfov / output_vfov
      self.output_projection   = 
         projection.SphericalProjection.new(output_width,output_height,output_hfov,output_vfov)
   end
   return self.output_projection
end

function SweepImageAligner:get_pair_mapper(force)
   if (not self.pair_mapper) or force then
      input_projection = self:get_input_projection(force)
      pair_projection  = self:get_pair_projection(force)
   
      self.pair_mapper = 
         projection.Remap.new(input_projection,pair_projection)
   end
   return self.pair_mapper
end

function SweepImageAligner:get_output_mapper(force)
   if (not self.output_mapper) or force then 
      input_projection = self:get_input_projection(force)
      output_projection = self:get_output_projection(force)

      self.output_mapper = 
         projection.Remap.new(input_projection,output_projection)
   end
   return self.output_mapper
end

-- update projections, height and width
function SweepImageAligner:set_scale(scale)
   force = true
   self.scale = scale
   self:get_input_width_height(force)
   self:get_input_radians_per_pixel(force)
   -- recreate the projections
   self:get_pair_mapper(force)
   self:get_output_mapper(force)
end

-- get size of first image
function SweepImageAligner:get_input_width_height(force )
   if not (self.input_width and self.input_height) or force then 
      if self.images then 
         self.input_width  = images[1]:size(3)
         self.input_height = images[1]:size(2)
      else
         self.image_wand:load(self.image_filenames[1])
         w,h               = image_wand:size()
         self.input_height = math.floor(h*self.scale)
         self.input_width  = math.floor(w*self.scale)
         image_wand:size(self.input_width,self.input_height)
      end
   end
   return self.input_width, self.input_height
end

-- TODO if images are stored need to index by colorspace and size
function SweepImageAligner:get_image(i,colorspace)
   colorspace = colorspace or "RGB"
   img = nil
   if self.images and self.images[i] then
      img =  self.images[i]
   else
      w,h = self:get_input_width_height()
      self.image_wand:load(self.image_filenames[i])
      self.image_wand:size(w,h)
      img = self.image_wand:toTensor('float',colorspace,'DHW')
   end
   if self.store_images then 
      self.images = self.images or {}
      self.images[i] = img
   else
      -- only reason you wouldn't store is memory issues
      collectgarbage() 
   end 
   return img
end

-- this is the minmal stopping point
function SweepImageAligner:get_input_radians_per_pixel(force)
   if not self.input_radians_per_pixel or force then
      width, height = self:get_input_width_height()
      rad_per_px_x = self.hfov / (width + 1)
      rad_per_px_y = self.vfov / (height + 1)
      self.input_radians_per_pixel = math.min(rad_per_px_x, rad_per_px_y)
   end
   return self.input_radians_per_pixel
end

-- functions aren't part of the class(no dependencies on self so that they can be passed to optimize() function.
function score_projections(projected_image_left,mask_left,projected_image_right,mask_right)
   overlap_mask = mask_right:cmul(mask_left) 
   area         = overlap_mask:sum(); 
   image_diff   = projected_image_left - projected_image_right 
   image_dist   = image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float()) 
   return image_dist:sum()/area;
end

function compute_right_projection_and_score(mapper,image_right,projected_image_left,mask_left)
   mapper:update()
   projected_image_right = mapper:remap(image_right)
   mask_right            = mapper:get_alpha()
   return 
      score_projections(projected_image_left,mask_left,
                        projected_image_right,mask_right)
end

-- TODO do with a closure (reduce number of parameters
function update_input_lambda_and_score(lambda,mapper,phi,image_right,
                                       projected_image_left,mask_left)
   mapper:set_input_lambda_phi(lambda,phi)
   return 
      compute_right_projection_and_score(mapper,image_right,
                                         projected_image_left,mask_left)
end

-- find horizontal offsets 
function SweepImageAligner:find_best_lambda()
   n_images   = self.n_images
   wiggle     = self.lambda_wiggle_base
   stop       = self:get_input_radians_per_pixel()
   delta      = self.delta
   scores     = self.scores
   -- go through all pairs images find lambda. Start last to first
   previ      = n_images
   image_left = self:get_image(previ)
   
   mapper = self:get_pair_mapper()
   -- do optimization for each pair
   for i = 1,n_images do
      -- setup left image is at negative current best offset.
      left_lambda = -delta[1][i]
      left_phi    =  delta[2][previ]
      mapper:set_input_lambda_phi(left_lambda,left_phi)
      mapper:update()
      mask_left = mapper:get_alpha()
      projected_image_left = mapper:remap(image_left)

      -- load right image and set size
      image_right = self:get_image(i)

      -- difference from current best guess lambda
      lambda_delta  = 0
      right_phi     = delta[2][i]

      mapper:set_input_lambda_phi(lambda_delta,right_phi)
      mapper:update()

      projected_image_right = mapper:remap(image_right)
      mask_right            = mapper:get_alpha()
      
      current_best = 
         update_input_lambda_and_score(lambda_delta, 
                                       mapper, right_phi, image_right,
                                       projected_image_left,mask_left)

      lambda_delta, score = 
         optimize(wiggle,stop,
                  lambda_delta,current_best,
                  update_input_lambda_and_score,
                  mapper,right_phi,image_right,
                  projected_image_left,mask_left)

      delta[1][i] = delta[1][i] + lambda_delta
      scores[i]   = score;
      printf(" - best lambda found for %d: %2.4f score: %2.4f", i, delta[1][i], score);
      
      image_left = image_right
      previ      = i
   end
end

-- function compute_phi_score(current_phi, image_files, delta)

--    -- assume that phi will be the same for all images as this
--    -- represents the camera not looking exactly at the plane
--    -- of rotation, but slightly above or below.
--    score = 0
--    image_wand:load(image_files[#image_files])
--    image_wand:size(width,height)
--    image_left = image_wand:toTensor('float',"RGB","DHW")
--    previ = #image_files
--    for i = 1,#image_files do
--       collectgarbage()
--       lambda_left  = -delta[1][i]
--       lambda_right = 0 
--       input_projection:set_lambda_phi(lambda_left,current_phi)
--       _,_,mask_left = self.pair_mapper:get_offset_and_mask(force)
--       mask_left = mask_left:eq(0);
        
--       projected_image_left = self.pair_mapper:remap(image_left)
--       image_wand:load(image_files[i])
--       image_wand:size(width,height)
--       image_right = image_wand:toTensor('float',"RGB","DHW")
            
--       input_projection:set_lambda_phi(lambda_right,current_phi)
--       _,_,mask_right = self.pair_mapper:get_offset_and_mask(force)
--       projected_image_right = self.pair_mapper:remap(image_right)
                        
--       mask_right = mask_right:eq(0)
--       overlap_mask = mask_right:cmul(mask_left)
--       area = overlap_mask:sum()

--       image_diff = projected_image_left - projected_image_right
            
--       image_dist = image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
--       if area <= 0 then
--          print("Warning no overlap this should not happen")
--       else
--          score = score + image_dist:sum()/area;
--       end
--       image_left = image_right
--       mask_left  = mask_right
--    end
--    return score
-- end

-- function find_best_phi(image_files, delta, wiggle, stop, current_phi,current_best_score)
--    wiggle      = wiggle or phi_wiggle_base
--    stop        = stop or rad_per_pixel
--    current_phi = current_phi or 0
--    best_phi    = current_phi 
--    current_best_score = current_best_score or compute_phi_score(current_phi,image_files,delta)
--    printf("Adjusting global phi: %f", current_phi)
--    test_phi, score = 
--       optimize(wiggle,stop,
--                best_phi,current_best_score,
--                compute_phi_score,
--                image_files,delta)
   
--    if score < current_best_score then
--       best_phi = test_phi
--       current_best_score = score
--    end
--    delta[2]:fill(best_phi)
--    printf(" - best phi found %2.4f %2.4f", best_phi, current_best_score);
--    return best_phi, current_best_score
-- end

-- function compute_vfov_score(current_vfov, image_files, delta)

--    test_input_projection = 
--       projection.GnomonicProjection.new(width,height,hfov,current_vfov)
   
--    test_rect_to_sphere = projection.Remap.new(test_input_projection,pair_proj_to)

--    printf(" -     projection_from: hfov: %2.4f vfov: %2.4f",
--           test_rect_to_sphere.projection_from.hfov,
--           test_rect_to_sphere.projection_from.vfov)


--    score = 0
--    image_wand:load(image_files[#image_files])
--    image_wand:size(width,height)
--    image_left = image_wand:toTensor('float',"RGB","DHW")
--    previ = #image_files
--    for i = 1,#image_files do
--       collectgarbage()
--       lambda_left  = -delta[1][i]
--       phi_left     = delta[2][previ]
--       lambda_right = 0 
--       phi_right    = delta[2][i]

--       test_input_projection:set_lambda_phi(lambda_left,phi_left)
--       _,_,mask_left = test_rect_to_sphere:get_offset_and_mask(force)
--       mask_left = mask_left:eq(0);
        
--       projected_image_left = test_rect_to_sphere:remap(image_left)
--       image_wand:load(image_files[i])
--       image_wand:size(width,height)
--       image_right = image_wand:toTensor('float',"RGB","DHW")
            
--       test_input_projection:set_lambda_phi(lambda_right,phi_right)
--       _,_,mask_right = test_rect_to_sphere:get_offset_and_mask(force)
--       projected_image_right = test_rect_to_sphere:remap(image_right)
                        
--       mask_right   = mask_right:eq(0)
--       overlap_mask = mask_right:cmul(mask_left)
--       area         = overlap_mask:sum()

--       image_diff = projected_image_left - projected_image_right
            
--       image_dist = 
--          image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
--       if area <= 0 then
--          print("Warning no overlap this should not happen")
--       else
--          score = score + image_dist:sum()/area;
--       end
--       image_left = image_right
--       mask_left  = mask_right
--    end
--    return score
-- end

-- function find_best_vfov(image_files, delta, wiggle, stop, current_vfov,current_best_score)
--    wiggle       = wiggle or phi_wiggle_base
--    stop         = stop or rad_per_pixel
--    current_vfov = current_vfov or vfov
--    best_vfov    = current_vfov
--    current_best_score = current_best_score or 
--       compute_vfov_score(current_vfov,image_files,delta)
--    printf("Adjusting vfov: %f",current_vfov)
--    test_vfov, score = 
--       optimize(wiggle,stop,
--                current_vfov,current_best_score,
--                compute_vfov_score,
--                image_files,delta)
   
--    if score < current_best_score then
--       best_vfov = test_vfov
--       current_best_score = score
--    end
   
--    printf("new best vfov found %2.4f %2.4f", best_vfov, current_best_score);
--    return best_vfov, current_best_score
-- end

-- function compute_hfov_score(current_hfov, image_files, delta)

--    -- TODO this is the only difference between compute_hfov and vfov,
--    -- perhaps the projection itself should be the parameter to a
--    -- compute_projection_score ?
--    test_input_projection = 
--       projection.GnomonicProjection.new(width,height,current_hfov,vfov)
   
--    test_rect_to_sphere = projection.Remap.new(test_input_projection,pair_proj_to)

--    printf(" -     projection_from: hfov: %2.4f vfov: %2.4f",
--           test_rect_to_sphere.projection_from.hfov,
--           test_rect_to_sphere.projection_from.vfov)

--    score = 0
--    image_wand:load(image_files[#image_files])
--    image_wand:size(width,height)
--    image_left = image_wand:toTensor('float',"RGB","DHW")
--    previ = #image_files
--    for i = 1,#image_files do
--       collectgarbage()
--       lambda_left  = -delta[1][i]
--       phi_left     = delta[2][previ]
--       lambda_right = 0 
--       phi_right    = delta[2][i]

--       test_input_projection:set_lambda_phi(lambda_left,phi_left)
--       _,_,mask_left = test_rect_to_sphere:get_offset_and_mask(force)
--       mask_left = mask_left:eq(0);
        
--       projected_image_left = test_rect_to_sphere:remap(image_left)
--       image_wand:load(image_files[i])
--       image_wand:size(width,height)
--       image_right = image_wand:toTensor('float',"RGB","DHW")
            
--       test_input_projection:set_lambda_phi(lambda_right,phi_right)
--       _,_,mask_right = test_rect_to_sphere:get_offset_and_mask(force)
--       projected_image_right = test_rect_to_sphere:remap(image_right)
                        
--       mask_right   = mask_right:eq(0)
--       overlap_mask = mask_right:cmul(mask_left)
--       area         = overlap_mask:sum()

--       image_diff = projected_image_left - projected_image_right
            
--       image_dist = 
--          image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
--       if area <= 0 then
--          print("Warning no overlap this should not happen")
--       else
--          score = score + image_dist:sum()/area;
--       end
--       image_left = image_right
--       mask_left  = mask_right
--    end
--    return score
-- end

-- function find_best_hfov(image_files, delta, wiggle, stop, current_hfov,current_best_score)
--    wiggle       = wiggle or phi_wiggle_base
--    stop         = stop or rad_per_pixel
--    current_hfov = current_hfov or hfov
--    best_hfov    = current_hfov
--    current_best_score = current_best_score or 
--       compute_hfov_score(current_hfov,image_files,delta)
--    printf("Adjusting hfov: %f",current_hfov)
--    test_hfov, score = 
--       optimize(wiggle,stop,
--                current_hfov,current_best_score,
--                compute_hfov_score,
--                image_files,delta)
   
--    if score < current_best_score then
--       best_hfov = test_hfov
--       current_best_score = score
--    end
   
--    printf("new best hfov found %2.4f %2.4f", best_hfov, current_best_score);
--    return best_hfov, current_best_score
-- end
   
-- find_best_lambda(image_files, delta, lambda_wiggle_base, rad_per_pixel)
-- best_score = scores:sum()
-- printf("current best_score: %f", best_score)

-- best_phi, best_score = find_best_phi(image_files, delta, phi_wiggle_base, rad_per_pixel, 0, best_score)
-- printf("current best_score: %f", best_score)

-- -- best_vfov, best_score = find_best_vfov(image_files, delta, vfov*0.05, rad_per_pixel, vfov, best_score)
-- -- printf("current best_score: %f", best_score)
-- -- input_projection:set_vfov(best_vfov)
-- -- vfov = best_vfov

-- -- best_hfov, best_score = find_best_hfov(image_files, delta, hfov*0.05, rad_per_pixel, hfov, best_score)
-- -- printf("current best_score: %f", best_score)
-- -- input_projection:set_hfov(best_hfov)
-- -- hfov = best_hfov

-- -- find_best_lambda(image_files, delta, 2*rad_per_pixel, rad_per_pixel)
-- -- best_score = scores:sum()
-- -- print(scores)
-- -- printf("current best_score: %f", best_score)

-- -- display
-- _G.mapped_image_files = {}
-- _G.masks = {}
-- _G.out_fnames = ""

function SweepImageAligner:display_and_save(enblend,outfname)
   outdir      = path.dirname(outfname)
   outbasename = path.basename(outfname)
   tmp_fnames  = ""

   n_images    = self.n_images
   delta       = self.delta
   mapper      = self:get_output_mapper()

   mapped_image_files = {}
   masks              = {}

   lambda = delta[1][1];
   phi    = delta[2][1]

   for i = 1,n_images do

      img = self:get_image(i,"RGBA")

      mapper:set_input_lambda_phi(lambda,phi)
      mapper:update()
      mask = mapper:get_mask()
      alpha = mapper:get_alpha()

      img_out = mapper:remap(img)
      img_out[4]:copy(alpha:mul(255))

      table.insert(mapped_image_files,img_out)
      table.insert(masks,mask)

      if enblend then
         tmp_name = string.format("tmp_%02d.png", i)
         self.image_wand:fromTensor(img_out,"RGBA","DHW")
         printf(" - saving %s %s",tmp_name, image_wand:imagetype())
        image_wand:save(tmp_name)
         tmp_fnames = tmp_fnames .. " " .. tmp_name
      end
      
      if i<n_images then
         lambda = lambda + delta[1][i+1]
         phi = delta[2][i+1]
      end
   end
   allimg = blend(mapped_image_files, masks)

   image.display(allimg)


   printf("saving %s", outfname)
   image.save(string.format("%s",outfname), allimg)

   if (enblend) then
      os.execute(string.format("enblend %s -o %s/enblend_%s", tmp_fnames, outdir, outbasename))
      os.execute(string.format("rm %s", tmp_fnames))
   end
end
