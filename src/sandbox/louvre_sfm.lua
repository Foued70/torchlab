fs       = require 'fs'
path     = require 'path'
opencv   = require "../opencv/init"
saliency = require '../image/saliency'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Align images in a sweep')
cmd:text()
cmd:text('Options')
cmd:option('-images_dir',
           "/Users/marco/Google Drive/Louvre/large_size",
           'base directory containing images')
cmd:option('-output_dir',
           "",
           'base directory containing images')
cmd:text()

-- parse input params
params         = cmd:parse(process.argv)
images_dir     = params.images_dir
output_dir     = params.output_dir
if output_dir == "" then 
   output_dir = images_dir.. "/output/"
end
print("saving files to ".. output_dir)
util.fs.mkdir_p(output_dir)

detector_type  = "BRISK"
extractor_type = "SIFT"
matcher_type   = "FlannBased"
npts           = 2500

_G.files       = util.fs.glob(images_dir,"png")


if test_detectors then
   detector_tbl = {}
   for detector_type,val in pairs(opencv.Detector.types) do 
      detector_tbl[val] =  opencv.Detector.new(detector_type)
   end
else
   detector     = opencv.Detector.new(detector_type)
end
extractor       = opencv.Extractor.new(extractor_type)
matcher         = opencv.Matcher.new(matcher_type)

_G.images_tbl   = {}
_G.keypoint_tbl = {}
_G.features_tbl = {}

for _,fpath in pairs(files) do
   if fs.existsSync(fpath) and fs.statSync(fpath).is_file then
      print("Loading ".. fpath)
      wand = image.Wand.new(fpath)
      -- img_BGR = wand:toTensor("byte","RGB","HWD",true)
      img_th  = wand:toTensor("byte","LAB","DHW",true)

      -- img_sal = saliency.high_entropy_features(wand:toTensor("double","LAB","DHW",true))

      -- collectgarbage()
      -- image.save(fpath.."-saliency.png",img_sal)
      -- image.display(img_sal)

      -- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
      img_mat  = opencv.Mat.new(img_th[1])
      
      if test_detector then 
         draw_mat = opencv.Mat.new(fpath)
         for detector_type,val in pairs(opencv.Detector.types) do 
            detector = detector_tbl[val]
            kpts_found, npts_found    = detector:detect(img_mat,npts)
            print(" + "..detector_type.." found : "..npts_found)
            
            this_draw_mat = draw_mat:clone()
            draw_mat:convert(this_draw_mat,"BGR2RGB")
            opencv.utils.draw_keypoints(this_draw_mat,kpts_found,npts_found)
            
            -- this_draw_mat:display(fpath.." "..detector_type)
            fname = fpath:gsub(".png", "-"..detector_type..".jpg")
            fname = output_dir .. "/"..path.basename(fname)
            print("Saving: " .. fname)
            log.tic()
            image.save(fname,this_draw_mat:toTensor("DHW"))
            print(" + in " .. log.toc())
            collectgarbage()
         end
      else
         kpts_found, npts_found    = detector:detect(img_mat,npts)
         print(" + "..detector_type.." found : "..npts_found)
         descriptors  = extractor:compute(img_mat,kpts_found,npts_found)
         -- table.insert(images_tbl,   img_mat)
         table.insert(keypoint_tbl, { kpts_found, npts_found } )
         table.insert(features_tbl, descriptors)
      end 
      collectgarbage()
   end
end

_G.matches_tbl = {}
for i = 1,#features_tbl do 
   descriptors_src = features_tbl[i]
   size_src        = descriptors_src:size()[1]
   matches_tbl[i] = {}
   for j = i+1,#features_tbl do 
      descriptors_dest = features_tbl[j]
      size_dest        = descriptors_dest:size()[1]

      matches,nmatches = matcher:match(descriptors_src, descriptors_dest, size_src*size_dest)

      matches_good,nmatches_good = matcher:reduce(matches, nmatches)

      print( i .. " -> " ..j.." found "..nmatches_good.." matches")
      if nmatches_good > npts*0.7 then 
         print(" + keeping ...")
         matches_tbl[i][j] = {matches_good, nmatches_good}

         --      H = opencv.calib3d.getHomography(kpts_src, npts_src, kpts_dest, npts_dest, matches, nmatches)

      end
   end
end

