function test_offset_to_pixel_coords(debug)
   -- 120 has long list of divisors, to test 1D to 2D
   local index = torch.LongTensor():range(0,119) -- C index
   local rw = torch.Tensor({2,3,4,5,6,8,10,12,15,20,24,30,40,60})
   local toterr = 0
   for r = 1,rw:size(1) do 
      local stride    = torch.Tensor({rw[r],1})
      local coords    = util.addr.offset_to_pixel_coords(index,stride)
      local index_out = util.addr.pixel_coords_to_offset(coords,stride)
      -- compute errors
      local err = index - index_out
      local err_int = err:ne(0):sum()
      toterr = toterr + err_int
      if debug and (err_int > 0) then 
         printf("ERRORS: %d (stride: %d,%d)",err_int,stride[1],stride[2])
         for ii = 1,120 do 
            if index_out[ii] ~= index[ii] then
               printf("c: %d,%d iout: %d ~= i: %d",
                      coords[1][ii],coords[2][ii],index_out[ii],index[ii])
            end 
         end
      end
   end
   printf("Errors: %d/%d",toterr,120*rw:size(1))

   local rw = {{2,3,20},{3,4,10},{4,5,6},{5,6,4},{8,3,5}}
   local toterr = 0
   for r = 1,#rw do 
      local size      = torch.LongTensor(rw[r])
      local stride    = torch.Tensor(size:storage()):stride()
      local coords    = util.addr.offset_to_pixel_coords(index,stride)
      local index_out = util.addr.pixel_coords_to_offset(coords,stride)
      -- compute errors
      local err = index - index_out
      local err_int = err:ne(0):sum()
      toterr = toterr + err_int
      if debug and (err_int > 0) then 
         printf("ERRORS: %d (stride: %d,%d)",err_int,stride[1],stride[2])
         for ii = 1,120 do 
            if index_out[ii] ~= index[ii] then
               printf("c: %d,%d iout: %d ~= i: %d",
                      coords[1][ii],coords[2][ii],index_out[ii],index[ii])
            end 
         end
      end
   end
   printf("Errors: %d/%d",toterr,120*#rw)
end

test_offset_to_pixel_coords()
   
