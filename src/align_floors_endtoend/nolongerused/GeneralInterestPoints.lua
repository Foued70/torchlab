GeneralInterestPoints = Class()
GeneralInterestPoints.dtype = "SIFT"
GeneralInterestPoints.npts_interest = 1000

function __init(parameters)
	for paramName in pairs(parameters) do
      if(GeneralInterestPoints[paramName]) then
         GeneralInterestPoints[paramName] = parameters[paramName]
      else
         error("invalid initializing of GeneralInterestPoints with variable" .. paramName)
      end
   end
end

function GeneralInterestPoints:getPoints(img_src, img_dest)
   detector    = opencv.Detector.new(GeneralInterestPoints.dtype)
   kpts_src, npts_src  = detector:detect(img_src,GeneralInterestPoints.npts_interest)
   kpts_dest, npts_dest  = detector:detect(img_dest,GeneralInterestPoints.npts_interest)
   
   src_corners = torch.Tensor(npts_src,2)
   dest_corners = torch.Tensor(npts_dest,2)

   for i=1, npts_src do
      src_x = torch.ceil(kpts_src[i].pt.x)
      src_y = torch.ceil(kpts_src[i].pt.y)

      src_corners[i][1] = src_y
      src_corners[i][2] = src_x
   end
   for i=1, npts_dest do
      dest_x = torch.ceil(kpts_dest[i].pt.x)
      dest_y = torch.ceil(kpts_dest[i].pt.y)
      dest_corners[i][1] = dest_y
      dest_corners[i][2] = dest_x
   end
   return src_corners, dest_corners
end