test_slow = false

if test_slow then
   if (not xmap or not ymap) then
      print("Error can't test need xmap and ymap which were made local")
   else
      function remap_xy(img,xmap,ymap)
         local out = torch.Tensor(img:size(1),xmap:size(1),xmap:size(2))
         for yi = 1,xmap:size(1) do
            for xi = 1,xmap:size(2) do 
               out[1][yi][xi] = img[1][ymap[yi][xi]][xmap[yi][xi]]
               out[2][yi][xi] = img[2][ymap[yi][xi]][xmap[yi][xi]]
               out[3][yi][xi] = img[3][ymap[yi][xi]][xmap[yi][xi]]
            end
         end
         return out
      end
      
      sys.tic()
      output_image_slow = remap_xy(img,xmap,ymap)
      printf("time slow: %2.4fs",sys.toc())
      print("Testing against slow version")
      img1 = img[1]
      img1f = img[1]
      img1f:resize(img1:size(1)*img1:size(2))
      sqmap = equimap:clone()
      sqmap:resize(xmap:size(1),xmap:size(2))
      imgw = img:size(3)
      err = 0
      cnt = 0
      for i = 1,img:size(2),100 do
         for j = 1,img:size(3),100 do 
            cnt = cnt + 1
            local xm = math.floor(xmap[i][j] + 0.5)
            local ym = math.floor(ymap[i][j] + 0.5)
            local im = math.floor((ym - 1)*1024 + xm + 0.5)
            local sq = sqmap[i][j]
            local em = equimap[(i-1)*1024 + j]
            local im1 = img1[ym][xm]
            local im2 = img1f[im]
            local im3 = img1f[sq]
            local dif = im - sq
            if (torch.abs(dif) > 0) then
               printf("[%3d][%3d] x: %4d y: %4d img: %f",
                      i,j, xm,ym, im1)
               printf("   im: %d  sq: %d eq: %d diff: %d img: %f %f",
                      im, sq, em, im - sq, im2, im3) 
               printf(" ERRROR: %d",dif)
               err = err + 1
            end
         end    
      end
      printf("Err: %d/%d",err,cnt)
   end
end
