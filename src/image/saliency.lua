-- load C lib
local libsaliency = require 'libsaliency'

function high_entropy_features (...)
   local _, img, kr, kc, sr, sc, 
        nscale, scalefactor, nbins, 
        entropyType, nmswinsize, npts = dok.unpack(
      {...},
      'high_entropy_features',
      [[ 
Implements a saliency detector based on entropy of histogram of input.
And searches for a saliency across scales.
]],
      {arg='img', type='torch.Tensor',
       help='torch.Tensor image (though can have multiple channels', 
       req=true},
      {arg='kr', type='number',
       help='size of hist window in pixels (row)', default=5},
      {arg='kc', type='number',
       help='size of hist window in pixels (col)', default=5},
      {arg='sr', type='number',
       help='step of hist window in pixels (row)', default=1},
      {arg='sc', type='number',
       help='step of hist window in pixels (col)', default=1}, 
      {arg='nscale', type='number',
       help='how many scales?', default=3},
      {arg='scalefactor', type='number',
       help='what is scaling factor between scales', default=1.2},
      {arg='nbins', type='number',
       help='how many bins in the histograms ?', default=8},
      {arg='entropyType', type='string',
       help='compute true entropy or fast approx (entropy or meanOverMax)',
       default='meanovermax'}
   )

   local t = torch.Timer()
   if img:dim() == 2 then 
      img = img:reshape(1,img:size(1),img:size(2))
   end
   local w  = img:size(3)
   local h  = img:size(2)
   local ii = torch.Tensor(img:size(1),h,w,nbins)
   local rr = torch.Tensor(img:size(1),h-kr+1,w-kc+1,nbins)
   local sm = torch.Tensor(nscale,h,w):zero()
   local m  = torch.Tensor(h,w):zero()

   -- compute once for all images
   local tTotal = t:time().real
   
   torch.Tensor.libsaliency.intHistPack(ii,img,nbins)
   local tintHist = t:time().real - tTotal
   local tAvg = {}
   local tEnt = {}
   -- FIXME optimize this make a multi-scale in C.
   for i = 1,nscale do
      tAvg[i] = t:time().real 
      local lkr = math.floor(kr * scalefactor^(i-1))
      local lkc = math.floor(kc * scalefactor^(i-1))
      print(' - computing ('..lkr..','..lkc..')')
      torch.Tensor.libsaliency.intAvg(rr,ii,lkr,lkc,sr,sc)
      tAvg[i] = t:time().real - tAvg[i]
      tEnt[i] = t:time().real
      local nch = rr:size(1)
      local nr  = rr:size(2)
      local nc  = rr:size(3)
      local outsizer = math.floor(0.5+(h-lkr+1)/sr)
      local outsizec = math.floor(0.5+(w-lkc+1)/sc)
      local fm  = torch.Tensor(outsizer,outsizec)
      if entropyType == 'entropy' then 
         torch.Tensor.libsaliency.spatialEnt(fm,rr);
      elseif entropyType == 'MeanOver' then
         torch.Tensor.libsaliency.spatialMeanOverMax(fm,rr);
      else 
         torch.Tensor.libsaliency.spatialOneOverMax(fm,rr);
      end
      -- print("["..i.."] min: "..fm:min().." max: "..fm:max())
      sm:select(1,i):narrow(1,math.floor(lkr/2),outsizer):narrow(2,math.floor(lkc/2),outsizec):copy(fm)
      tEnt[i] = t:time().real - tEnt[i] 
   end
   local tScaleSaliency = t:time().real
   -- compute saliency across scales
   if nscale == 1 then
      m = sm:select(1,1)
   else
      m.libsaliency.scaleSaliency(m,sm)
      -- y = H*S
      m:cmul(sm:sum(1):select(1,1))
   end
   tScaleSaliency = t:time().real - tScaleSaliency
   local tclearB = t:time().real
   local mkr = math.floor(kr*scalefactor^(nscale-1))
   local mkc = math.floor(kc*scalefactor^(nscale-1))
   local okr = math.floor(mkr/2)
   local okc = math.floor(mkc/2)
   local opr = math.floor(0.5+(h-mkr+1)/sr)
   local opc = math.floor(0.5+(w-mkc+1)/sc)
   local tmpm  = torch.Tensor(m:size()):zero()
   tmpm:narrow(1,okr,opr):narrow(2,okc,opc):fill(1)
   -- clear the borders
   m:cmul(tmpm)
   tclearB = t:time().real - tclearB
   tTotal = t:time().real - tTotal
   print(string.format(" - intHist %2d bins: %2.3fs (%2.1f%%)",
                       nbins,tintHist,tintHist/tTotal*100))
   print(" - scales:")
   for i = 1,nscale do
      print(string.format(" -- SpatialAvg[%d]:  %2.3fs (%2.1f%%)",
            i,tAvg[i],tAvg[i]/tTotal*100))
      print(string.format(" -- SpatialEnt[%d]:  %2.3fs (%2.1f%%)",
            i,tEnt[i],tEnt[i]/tTotal*100))
   end
   print(string.format(" - scale saliency:  %2.3fs (%2.1f%%)",
                       tScaleSaliency,tScaleSaliency/tTotal*100))
   print(string.format(" - clear border:  %2.3fs (%2.1f%%)",
                       tclearB,tclearB/tTotal*100))
   print(string.format("       Total time:  %2.3fs (%2.1f%%)",
                       tTotal,tTotal/tTotal*100))
   return m,sm
end

-- sm is a 3D tensor nscales x width x height. This function sums the
-- abs difference between the scales and multiplies this by the sum of
-- the saliencies at each scale.
function scaleSaliency(sm)
   local m = torch.Tensor(sm:size(2),sm:size(3))
   -- compute saliency across scales
   if sm:size(1) == 1 then
      m = sm:select(1,1)
   else
      m.libsaliency.scaleSaliency(m,sm)
      -- y = H*S
      m:cmul(sm:sum(1):select(1,1))
   end
   return m
end


return {
   high_entropy_features = high_entropy_features
}
