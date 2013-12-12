
CornerHarris = Class()
CornerHarris.threshold = 70 --anything w score > .1*max score will be kept
CornerHarris.blockSize =2
CornerHarris.kSize =3
CornerHarris.k = .04
CornerHarris.npts_interest = 1000
CornerHarris.radius_local_max = 12;

function CornerHarris:__init(parameters)
   for paramName in pairs(parameters) do
      if(CornerHarris[paramName]) then
         CornerHarris[paramName] = parameters[paramName]
      else
         error("invalid initializing of CornerHarris with variable" .. paramName)
      end
   end
end
--return 3xn matrix where each coordinate represents x,y,score of a key point using opencv's CornerHarris method
function CornerHarris:findCorners(img)
   local scoresMat  = opencv.imgproc.detectCornerHarris(img, CornerHarris.blockSize, CornerHarris.kSize, CornerHarris.k)
   local scoresTorchF = scoresMat:toTensor()
   local scoresTorchSquare = torch.DoubleTensor(scoresTorchF:size()):copy(scoresTorchF)
   
   local thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare, CornerHarris.threshold)]
   local goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare,CornerHarris.threshold)
   
   local scoresTorchSquare = CornerHarris.pickLocalMaxFast(scoresTorchSquare, goodLocationsX, goodLocationsY)
   local thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,CornerHarris.threshold)]
   local goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare, CornerHarris.threshold)
   local t1,sortOrder = torch.sort(thresholdScores,1,true)
   local topScores = torch.Tensor(math.min(CornerHarris.npts_interest,thresholdScores:size(1)),3)
   
   local tmp1 = img:toTensor():type('torch.DoubleTensor')
   local tmp1 = tmp1:clone():cdiv(tmp1:clone():add(0.00000001))
   
   for i=1,math.min(CornerHarris.npts_interest,thresholdScores:size(1))  do
   
   	  local cand = torch.Tensor({goodLocationsX[sortOrder[i]],goodLocationsY[sortOrder[i]],t1[i]})
	  
		topScores[{i,{}}]= cand
	  
	end

   return topScores;
end

--removes points which are not local max's in a radius_local_max bounding box around them
--only considers points which have been deemed as goods (i.e. larger than the threshold and in goodlocationsX/Y)
function CornerHarris.pickLocalMaxFast(scores_torch, goodLocationsX, goodLocationsY)
   local output = torch.zeros(scores_torch:size())
   i =0
   goodLocationsX:apply(function(val) 
      i=i+1
      x = goodLocationsX[i]
      y = goodLocationsY[i]
      localMax = scores_torch[{
         {math.max(x-CornerHarris.radius_local_max,1), math.min(x+CornerHarris.radius_local_max, scores_torch:size(1))},
         {math.max(y-CornerHarris.radius_local_max,1), math.min(y+CornerHarris.radius_local_max, scores_torch:size(2))}
         }]:max();
      if(scores_torch[x][y]>=localMax) then
         output[x][y] =scores_torch[x][y]
      end
   end)
   return output
end
--removes points which are not local max's in a radius_local_max bounding box around them
function CornerHarris.pickLocalMaxSlow(scores_torch)
   local output = scores_torch:clone()
   i =0
   output:apply(function(val) 
      i=i+1
      x = torch.ceil(i/scores_torch:size(2))
      y = (i-1)%scores_torch:size(2)+1
      localMax = scores_torch[{
         {math.max(x-CornerHarris.radius_local_max,1), math.min(x+CornerHarris.radius_local_max, scores_torch:size(1))},
         {math.max(y-CornerHarris.radius_local_max,1), math.min(y+CornerHarris.radius_local_max, scores_torch:size(2))}
         }]:max();
      if(scores_torch[x][y]>=localMax) then
         return val
      else
         return 0
      end
   end)
   return output
end