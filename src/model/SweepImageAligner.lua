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
      for _,fn in pairs(self.image_filenames) do 
         printf(" - file: %s",fn)
      end
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
      error("don't understand type of argument")
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

   scale        = 0.1
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

-- get size of first image
function SweepImageAligner:get_input_width_height(force_update)
   if (not (self.input_width and self.input_height)) or force_update then 
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


function SweepImageAligner:get_global_phi()
   return self.delta[2]:mean()
end

function SweepImageAligner:set_global_score(global_score)
   if global_score then 
      self.global_score = global_score
   else
      self.global_score = self.scores:sum()
   end
end

function SweepImageAligner:get_global_score()
   if not self.global_score then 
      self:set_global_score()
   end
   return self.global_score
end

function SweepImageAligner:get_input_projection(force_update)
   if (not self.input_projection) or force_update then 
      width,height = self:get_input_width_height()
      hfov = self.input_hfov
      hfov = self.input_hfov
      self.input_projection  = 
         projection.GnomonicProjection.new(width,height,hfov,vfov)
   end
   return self.input_projection
end

-- pair is the workspace for a pair of images
function SweepImageAligner:get_pair_projection(force_update)
   if (not self.pair_projection) or force_update then 
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

function SweepImageAligner:get_pair_mapper(force_update)
   if (not self.pair_mapper) or force_update then
      -- don't cascade the updates
      input_projection = self:get_input_projection()
      pair_projection  = self:get_pair_projection()
      self.pair_mapper = 
         projection.Remap.new(input_projection,pair_projection)
   end
   return self.pair_mapper
end

-- output full equirectangular
function SweepImageAligner:get_output_projection(force_update)
   if (not self.output_projection) or force_update then
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

function SweepImageAligner:get_output_mapper(force_update)
   if (not self.output_mapper) or force_update then 
      -- don't cascade updates
      input_projection = self:get_input_projection()
      output_projection = self:get_output_projection()

      self.output_mapper = 
         projection.Remap.new(input_projection,output_projection)
   end
   return self.output_mapper
end

-- update projections, height and width
function SweepImageAligner:set_scale(scale)
   force_update = true
   self.scale = scale
   -- recompute sizes
   self:get_input_width_height(force_update)
   self:get_input_radians_per_pixel(force_update)
   -- recreate the projections
   self:get_input_projection(force_update)
   self:get_pair_projection(force_update)
   self:get_output_projection(force_update)
   self:get_pair_mapper(force_update)
   self:get_output_mapper(force_update)
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

-- TODO do with a closure (reduce number of parameters)
function update_input_lambda_and_score(lambda,mapper,phi,image_right,
                                       projected_image_left,mask_left)
   mapper:set_input_lambda_phi(lambda,phi)
   return 
      compute_right_projection_and_score(mapper,image_right,
                                         projected_image_left,mask_left)
end

-- TODO do with a closure (reduce number of parameters)
function update_input_phi_and_score(phi,mapper,lambda,image_right,
                                    projected_image_left,mask_left)
   mapper:set_input_lambda_phi(lambda,phi)
   return 
      compute_right_projection_and_score(mapper,image_right,
                                         projected_image_left,mask_left)
end

-- find horizontal offsets (Pairwise)
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
      
      projected_image_left = mapper:remap(image_left)
      mask_left            = mapper:get_alpha()
      
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

      if score < scores[i] then
         delta[1][i] = delta[1][i] + lambda_delta
         printf(" - new lambda found for %d: %2.4f score: %2.4f < %2.4f", i, delta[1][i], score, scores[i]);
         scores[i]   = score 
      else
         printf(" - keeping lambda for %d: %2.4f score: %2.4f", i, delta[1][i], scores[i]);
      end
      image_left = image_right
      previ      = i
   end
   self:set_global_score()
   return delta[1], self:get_global_score(), scores
end

-- score current deltas
function SweepImageAligner:compute_scores(update)
   n_images   = self.n_images
   delta      = self.delta
   scores     = nil
   if (update) then
      scores = self.scores
   else
      -- not updating 
      scores     = torch.Tensor(n_images)
   end
   -- go through all pairs images find lambda. Start last to first
   previ      = n_images
   image_left = self:get_image(previ)
   
   mapper = self:get_pair_mapper()

   -- make sure we are using the best settings
   mapper:set_input_hfov_vfov(self.hfov,self.vfov)

   -- do optimization for each pair
   for i = 1,n_images do
      -- setup left image is at negative current best offset.
      left_lambda = -delta[1][i]
      left_phi    =  delta[2][previ]
      mapper:set_input_lambda_phi(left_lambda,left_phi)
      mapper:update()

      projected_image_left = mapper:remap(image_left)
      mask_left            = mapper:get_alpha()

      -- load right image and set size
      image_right = self:get_image(i)

      -- difference from current best guess lambda
      lambda_delta  = 0
      right_phi     = delta[2][i]

      mapper:set_input_lambda_phi(lambda_delta,right_phi)
      mapper:update()

      projected_image_right = mapper:remap(image_right)
      mask_right            = mapper:get_alpha()
      
      scores[i] = 
         update_input_lambda_and_score(lambda_delta, 
                                       mapper, right_phi, image_right,
                                       projected_image_left,mask_left)

      image_left = image_right
      previ      = i
   end
   if (update) then 
      self:set_global_score()
   end
   return scores:sum(), scores
end

function compute_global_phi_score(current_phi, aligner)

   -- assume that phi will be the same for all images as this
   -- represents the camera not looking exactly at the plane
   -- of rotation, but slightly above or below.
   n_images = aligner.n_images
   delta    = aligner.delta
   score    = torch.Tensor(n_images)
   mapper   = aligner:get_pair_mapper()

   image_left = aligner:get_image(n_images)

   previ = n_images
   for i = 1,n_images do
      collectgarbage()
      lambda_left  = -delta[1][i]
      lambda_right = 0 
      -- left 
      mapper:set_input_lambda_phi(lambda_left,current_phi)
      mapper:update()
      mask_left = mapper:get_alpha()
      projected_image_left = mapper:remap(image_left)

      -- right
      image_right = aligner:get_image(i)
            
      score[i] = 
         update_input_lambda_and_score(0,
                                       mapper, current_phi, image_right,
                                       projected_image_left,mask_left)
      
      image_left = image_right
   end
   return score
end

function SweepImageAligner:find_best_global_phi()

   wiggle      = self.phi_wiggle_base
   stop        = self:get_input_radians_per_pixel()
   delta       = self.delta
   scores      = self.scores

   current_phi = self:get_global_phi()
   best_phi    = current_phi 

   current_best_score = self:get_global_score()

   printf("Adjusting global phi: %f", current_phi)
   test_phi, score, scores = 
      optimize(wiggle,stop,
               best_phi,current_best_score,
               compute_global_phi_score,self)

   if score < current_best_score then
      best_phi = test_phi
      current_best_score = score
      self.scores = scores 
      self:set_global_score()
      printf(" - new phi found %2.4f %2.4f", best_phi, current_best_score);
      delta[2]:fill(best_phi)
   else
      printf(" - keeping phi  %2.4f %2.4f", best_phi, current_best_score);
   end
   return best_phi, current_best_score, self.scores
end

function compute_hfov_score(hfov,aligner)
   -- set hfov
   n_images = aligner.n_images
   delta    = aligner.delta
   mapper   = aligner:get_pair_mapper()
   mapper:set_input_hfov_vfov(hfov,aligner.vfov)
   return aligner:compute_scores()
end

function SweepImageAligner:find_best_input_hfov()

   wiggle       = self.hfov_wiggle_base
   stop         = self:get_input_radians_per_pixel()
   delta        = self.delta
   scores       = self.scores

   current_phi  = self:get_global_phi()
   current_hfov = self.hfov

   current_best_score = self:get_global_score()

   printf("Adjusting hfov: %f", current_hfov)
   test_hfov, score, scores = 
      optimize(wiggle,stop,
               current_hfov,current_best_score,
               compute_hfov_score,self)

   if score < current_best_score then
      self.hfov = test_hfov
      self.scores = scores
      self:set_global_score()
      current_best_score = score
      printf(" - new hfov found %2.4f %2.4f", hfov, current_best_score);
   else
      printf(" - keeping hfov %2.4f %2.4f", self.hfov, current_best_score);
   end

   -- leave mapper at best setting
   mapper:set_input_hfov_vfov(self.hfov,self.vfov)
   return self.hfov, current_best_score, self.scores
end

function compute_vfov_score(vfov,aligner)
   -- set vfov
   n_images = aligner.n_images
   delta    = aligner.delta
   mapper   = aligner:get_pair_mapper()
   mapper:set_input_hfov_vfov(aligner.hfov,vfov)
   return   aligner:compute_scores()
end

function SweepImageAligner:find_best_input_vfov()

   wiggle       = self.hfov_wiggle_base
   stop         = self:get_input_radians_per_pixel()
   delta        = self.delta
   scores       = self.scores

   current_vfov = self.vfov

   current_best_score = self:get_global_score()

   printf("Adjusting vfov: %f %f", current_vfov, current_best_score)
   test_vfov, score, scores = 
      optimize(wiggle,stop,
               current_vfov,current_best_score,
               compute_vfov_score,self)
   
   if score < current_best_score then
      self.vfov = test_vfov 
      self.scores = scores
      current_best_score = score
      self:set_global_score()
      printf(" - new vfov found %2.4f %2.4f", self.vfov, current_best_score);
   else
      printf(" - keeping vfov %2.4f %2.4f", self.vfov, current_best_score);
   end

   -- leave mapper at best setting
   mapper:set_input_hfov_vfov(self.hfov,self.vfov)

   return self.vfov, current_best_score, scores
end

function SweepImageAligner:generate_panorama(enblend,outfname, lambda)
   tmp_fnames  = ""
   n_images    = self.n_images
   delta       = self.delta
   mapper      = self:get_output_mapper()
   -- make sure we are using the best settings
   mapper:set_input_hfov_vfov(self.hfov,self.vfov)

   mapped_image_files = {}
   masks              = {}

   lambda = lambda or -pi
   phi    = delta[2][1]

   for i = 1,n_images do

      img = self:get_image(i,"RGBA")
     
      printf("[%d] lambda: %2.4f phi: %2.4f",i,lambda, phi)
      mapper:set_input_lambda_phi(lambda,phi)
      mapper:update()
      mask  = mapper:get_mask()
      alpha = mapper:get_alpha()

      img_out = mapper:remap(img)
      img_out[4]:copy(alpha:mul(255))

      table.insert(mapped_image_files,img_out)
      table.insert(masks,mask)

      if enblend then
         tmp_name = string.format("%s_tmp_%02d.png", outfname:gsub(".png",""), i)
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
   return blend(mapped_image_files, masks),tmp_fnames
end

function SweepImageAligner:display_and_save(enblend,outfname)
   outdir      = path.dirname(outfname)
   outbasename = path.basename(outfname)

   img,tmp_fnames = self:generate_panorama(enblend,outfname)
   image.display(img)
   
   printf("saving %s", outfname)
   image.save(string.format("%s",outfname), img)

   if (enblend) then 
      os.execute(string.format("enblend %s -o %s/enblend_%s", tmp_fnames, outdir, outbasename))
      os.execute(string.format("rm %s", tmp_fnames))
   end
end
