Wand  = image.Wand
fs    = require 'fs'
-- Make a lightfield

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Make horizontal and vertical slice light field')
cmd:text()
cmd:text('Options')
cmd:option('-frames_dir',
           "./",
           'base directory for sequence of frames')
cmd:option('-out_dir',
           "output",
           'base directory for output sequence of frames')
cmd:text()


-- parse input params
params = cmd:parse(process.argv)

d = params.frames_dir
outdir = d.."/"..params.out_dir

print("Saving to "..outdir)

save_frames = false
files       = util.fs.glob(d,{"jpg","JPG","jpeg","JPEG","png","PNG","hdr","tif","tiff"})
nfiles      = #files
max_size    = 1024
colorspace  = 'RGB'
tensor_type = 'torch.FloatTensor'
epi_batch_size  = 32
print("Processing ".. #files .. " potential files")
torch.setdefaulttensortype(tensor_type)
i = 0
for _,fpath in pairs(files) do
   -- fpath = d..f
   if fs.existsSync(fpath) and fs.statSync(fpath).is_file then
      i = i+1
      print("["..i.."/"..nfiles.."] "..fpath)
      local img = image.load(fpath,tensor_type,max_size,colorspace,'DHW')
      if (i == 1)  and not epi_image then
         height = img:size(2)
         width  = img:size(3)
         epi_image = torch.Tensor(nfiles,3,height,width)
      end      
      epi_image[i]:copy(img)
      print(" - min ".. img:min()..", max "..img:max())
      collectgarbage() 
   end
end
print("Found and loaded "..i.." images")

_G.epi_image = epi_image:narrow(1,1,i)

if not fs.existsSync(outdir) then fs.mkdirSync(outdir,"0775") end

nrows = epi_image:size(3) -- height
 
last = 1

for i = 1,nrows-epi_batch_size,epi_batch_size do
   -- clone ensures that we save the smaller storage, and that epi_row is contiguous
   epi_row = epi_image:narrow(3,i,epi_batch_size):transpose(1,2):clone()
   -- epi_row now: 3,nfiles,nrows,ncols
   file = string.format("%s/epi_image_%06d-%06d_%s.t7",outdir,i,i+epi_batch_size,tensor_type)
   print("saving: "..i.."/"..nrows.." ".. file)
   printf(" - %s (%d,%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3),epi_row:size(4))
   torch.save(file, epi_row)

   -- make the pictures look normal
   epi_row = epi_row:transpose(2,3):contiguous()
   epi_row:resize(epi_row:size(1),epi_row:size(2)*epi_row:size(3),epi_row:size(4))
   file = string.format("%s/epi_image_%06d-%06d_%s.png",outdir,i,i+epi_batch_size,tensor_type)
   print("saving: "..i.."/"..nrows.." ".. file)
   printf(" - %s (%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3))
   image.save(file,epi_row,colorspace,'DHW')

   last = last + epi_batch_size

   collectgarbage()

end

final_batch_size = nrows-last
print("last "..last.." final rows "..final_batch_size)
-- clone ensures that we save the smaller storage
epi_row = epi_image:narrow(3,last,final_batch_size):transpose(1,2):clone() 
file = string.format("%s/epi_image_%06d-%06d_%s.t7",outdir,last, nrows,tensor_type)
print("saving: ".. file)
printf(" - %s (%d,%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3),epi_row:size(4))
torch.save(file, epi_row)

-- save png for visual
epi_row = epi_row:transpose(2,3):contiguous()
epi_row:resize(epi_row:size(1),epi_row:size(2)*epi_row:size(3),epi_row:size(4))
file = string.format("%s/epi_image_%06d-%06d_%s.png",outdir,last,nrows,tensor_type)
print("saving: " .. file)
printf(" - %s (%d,%d,%d)", epi_row:type(), epi_row:size(1), epi_row:size(2), epi_row:size(3))
image.save(file,epi_row,colorspace,'DHW')

-- if make_movies then
--    -- make movies
--    ffmpeg_cmd = "ffmpeg -r 15 -i "..outdir.."/"..fname.."_%06d.png" ..
--       " -vcodec mjpeg -an "..
--       outdir.."/"..fname..".avi"
--    print(ffmpeg_cmd)
--    print(sys.execute(ffmpeg_cmd))
-- end
