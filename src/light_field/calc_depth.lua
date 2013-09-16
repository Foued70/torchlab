epi_file = 'rail_data/tone_mapped_512x339/output/epi_image_000001-000339_torch.FloatTensor.t7'

epi = torch.load(epi_file)
epi = epi:transpose(2,3):transpose(1,2):contiguous()
_G.epi = epi


epi_size    = epi:size()
nslice      = epi_size[1]
nchan       = epi_size[2]
nimg        = epi_size[3]
npx         = epi_size[4]

ninc        = npx / 4
nstep       = 1
mid_px      = 1

_G.sud       = torch.Tensor(ninc,npx)
_G.disparity = torch.Tensor(nslice,npx)

-- make_depth()
-- for each slice ...
for si = 1,nslice do
   epi_slice   = epi[si]
   nelem       = epi_slice:nElement()
   slice_size  = epi_slice:size()
   slice_index = torch.LongTensor():range(1,epi_slice:nElement())

   slice_index = slice_index:resize(epi_slice:size())
   cindex      = slice_index:clone()

   per_step    = torch.range(0,nimg-1):div(nimg):mul(0.5)
   epi_flat    = epi_slice:reshape(nelem)

   -- clear old sud
   sud:fill(0)

   printf("Slice %d",si)
   skew_time = 0
   kern_time = 0
   for inc = 1,ninc,nstep do
      log.tic()
      -- make skewed image
      per_img_offset = torch.mul(per_step,inc):long()

      incr_step = per_img_offset:resize(nimg,1):expand(nimg,npx)

      mask = torch.ByteTensor(slice_index:size())
      cindex:copy(slice_index)
      for c = 1,nchan do
         m = mask[c]
         s = cindex[c]
         smin = s:min(2):squeeze()
         smax = s:max(2):squeeze()
         s:add(-1,incr_step)
         for i = 1,nimg do
            torch.add(m[i],s[i]:lt(smin[i]),s[i]:gt(smax[i]))
         end
         s[m] = 1
      end

      cflat         = cindex:reshape(nelem)
      out_mat       = epi_flat[cflat]
      out_mat[mask] = 0

      skew_epi = out_mat:resize(slice_size)
      skew_time = skew_time + log.toc()
      log.tic()
      diff = skew_epi[{{},{mid_px},{}}]:expand(nchan,nimg,npx) - skew_epi
      -- Epanechnikov kernel as in paper
      -- http://en.wikipedia.org/wiki/Uniform_kernel#Kernel_functions_in_common_use
      u = diff:abs():sum(1):squeeze():mul(1/nchan)
      u:cmul(u):mul(-1):add(1):mul(3/4) -- 3/4(1 - u^2)
      sud[inc]:copy(u:sum(1):squeeze():mul(1/nimg))
      
      kern_time = kern_time + log.toc()
   end

   printf(" - skew %3.2fs kern: %3.2fs",skew_time*0.001, kern_time*0.001)

   max_val,disp = sud:max(1)
   mean_val = sud:mean(1)

   Cd = max_val - mean_val
   
   disp = disp:squeeze()

   printf(" - keeping %d/%d", Cd:ge(0.02):sum(), disp:nElement())

   disp[Cd:lt(0.02)] = 0
   
   disparity[si]:copy(disp)

   collectgarbage()

end
