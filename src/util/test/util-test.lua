require 'torch'
require 'sys'

local test = {}

function test.grid_contiguous()
   print("Testing grid contiguous")
   local ww  = 4
   local m   = torch.randn(32,32,3)
   local ufm = util.util.grid_contiguous(m,ww,ww)
   local err = 0
   local errc = 0
   for r = 1,ufm:size(1) do 
      for c = 1,ufm:size(2) do 
         local ow = m:narrow(1,1+(r-1)*ww,ww):narrow(2,1+(c-1)*ww,ww)            
         if 1e-8 < torch.sum(ufm[r][c] - ow) then 
            err = err + 1
         end
         if not ufm[r][c]:isContiguous() then
            errc = errc + 1
         end
      end
   end
   local ntests = ufm:size(1)*ufm:size(2)
   printf("-- %d/%d Errors/ %d/%d not contiguous ",err,ntests,errc,ntests)
end


function test.get_index_lt_val()
   local r = torch.sort(torch.randn(1000))
   local nr = r:size(1)
   
   function eval_test(r,val,verbose)
      local idx,count = util.util.get_index_lt_val(r,val)
      local err = 0
      if (idx > 1) then
         local cval = r[idx-1]
         local nval = r[idx]
         if (cval < val) and (nval >= val) then
            if verbose then
               printf("  OK idx: %d val: %f count: %d", idx, val, count)
            end
         else
            if verbose then
               printf("  ERROR idx: %d val: %f cval: %f nval: %f", idx, val, cval, nval)
            end
            errs = 1
         end
      else
         if (val <= r[1]) or (val > r[-1]) then
            if verbose then
               printf("  OK val: %f out of bounds",val)
            end
         else
            if verbose then
               printf("  ERROR idx: %d val: %f minr: %f maxr: %f", idx, val, r[1],r[nval])
            end
            errs = 1
         end
      end
      return err,count
   end

   print("Test 1 values in set")
   local steps = 0
   local errs  = 0
   
   for i = 1,nr do 
      local val = r[i]
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 2 jittered values")
   steps = 0
   errs  = 0
   
   for i = 1,nr do 
      local val = r[i] + torch.randn(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 3 bounds")
   steps = 0
   errs  = 0
   local rmin = r[1]
   local rmax = r[-1]
   for i = 1,nr*0.5 do 
      local val = rmin - torch.rand(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   for i = 1,nr*0.5 do 
      local val = rmax + torch.rand(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 4 duplicate + jittered values")
   steps = 0
   errs  = 0
   local uf = r:unfold(1,3,3)
   for i = 1,uf:size(1) do 
      uf[i]:fill(torch.randn(1)[1])
   end
   r = torch.sort(r)
   for i = 1,nr do 
      local val = r[i] + torch.randn(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f",errs,nr,steps/nr,nr)
   
end

function test.all()
   test.grid_contiguous()
   test.get_index_lt_val()
end

return test