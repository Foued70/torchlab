Wand  = image.Wand
fs    = require 'fs'

Epi = Class()

function Epi:__init(dirOrFiles, ...)
   self.tensor_type = "float"
   self.max_size    = 1024
   self.image_types = {"jpg","JPG","jpeg","JPEG","png","PNG","NEF","tif","tiff"}
   self.colorspace  = 'RGB'
   self.tensor_type = 'torch.FloatTensor'
   self.batch_size  = 512
   if type(dirOrFiles) == "string" and util.fs.is_dir(dirOrFiles) then
      self:load_dir(dirOrFiles)
   elseif type(dirOrFiles) == "table" then 
      self:load_files(dirOrFiles)
   else
      print("created empty Epi")
   end
end

function Epi:load_dir (dirname)
   files = util.fs.glob(dirname,self.image_types)
   self:load_files(files)
end

-- load_epi ()
function Epi:load_files(files)
   nfiles = #files
   print("Processing ".. nfiles .. " potential files")
   nimages = 0
   epi_image = nil
   for _,fpath in pairs(files) do
      -- fpath = d..f
      if fs.existsSync(fpath) and fs.statSync(fpath).is_file then
         nimages = nimages+1
         log.trace("["..nimages.."/"..nfiles.."] "..fpath)
         local img = image.load(fpath,tensor_type,max_size,colorspace,'DHW')
         if (nimages == 1)  and not self.epi_image then
            self.nchannels = img:size(1)
            self.height    = img:size(2)
            self.width     = img:size(3)
            epi_image = torch.Tensor(nfiles,self.nchannels,self.height,self.width)
         end
         epi_image[nimages]:copy(img)
         print(" - min ".. img:min()..", max "..img:max())
         collectgarbage()
      end
   end
   print("Found and loaded "..nimages.." images")
   self.nimages = nimages
   self.epi_image = epi_image:narrow(1,1,nimages)
end

function Epi:save_row(from,to,dir)
   dir = dir or self.cache_dir

   -- clone ensures that we save the smaller storage, and that epi_row is contiguous
   epi_row = self.epi_image[{{},{},{from,to},{}}]:clone()

   -- epi_row now: nfiles,3,nrows,ncols
   file = string.format("%s/epi_image_%06d-%06d_%s.t7",self.cache_dir,from,to,self.tensor_type)
   printf(" - %s (%d,%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3),epi_row:size(4))
   torch.save(file, epi_row)

   -- make the pictures look normal
   epi_row = epi_row:transpose(1,2):transpose(2,3):contiguous()
   epi_row:resize(epi_row:size(1),epi_row:size(2)*epi_row:size(3),epi_row:size(4))
   file = string.format("%s/epi_image_%06d-%06d_%s.png",self.cache_dir,from,to,self.tensor_type)
   printf(" - %s (%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3))
   image.save(file,epi_row,colorspace,'DHW')

end

function Epi:cache(dir)
   dir = dir or self.cache_dir
   self.cache_dir = dir

   print("Saving to "..dir)

   if not fs.existsSync(dir) then fs.mkdirSync(dir,"0775") end

   nrows      = self.height
   batch_size = self.batch_size
   last       = 1

   -- save slices
   print("Caching:")
   for i = 1,nrows-batch_size,batch_size do
      print(" - "..i.."/"..nrows)
      last = i + batch_size
      self:save_row(i,last,dir)
      collectgarbage()
   end

   if last < nrows then 
      print("last rows from "..last.." to  "..nrows)
      self:save_row(last,nrows,dir)
   end
end
