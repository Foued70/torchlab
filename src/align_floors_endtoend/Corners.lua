Corners = Class()

local harrisParameters = {}
harrisParameters.threshold = 75 --anything w score > .1*max score will be kept
harrisParameters.blockSize =5
harrisParameters.kSize =3
harrisParameters.k = .04
harrisParameters.npts_interest = 50
harrisParameters.radius_local_max = 10;

--HOUGH PARAMETERS
local HoughParameters = {}
HoughParameters.houghRo = 1
HoughParameters.houghTheta = math.pi/360
HoughParameters.houghMinLineLength = 25
HoughParameters.houghMaxLineGap = 80
HoughParameters.minThreshold = 10
HoughParameters.maxThreshold = 150
HoughParameters.defaultThreshold = 75
HoughParameters.numLinesDesired = 15

local GeneralInterestPointsParameters={}
GeneralInterestPointsParameters.dtype = "SIFT"
GeneralInterestPointsParameters.npts_interest = 150


--initialize based on parameters
parameterizedHough = align_floors_endtoend.Hough.new(HoughParameters)
local parameterizedHarris = align_floors_endtoend.cornerHarris.new(Harris)
local parameterizedGeneral = align_floors_endtoend.GeneralInterestPoints.new(GeneralInterestPointsParameters)

function Corners::getHarrisCorners(img_src, img_dest)
	return parameterizedHarris:findCorners(img_src)[{{}, {1,2}}],  parameterizedHarris:findCorners(img_dest)[{{}, {1,2}}]
end
 
 function Corners:getHoughCorners(img_src, img_dest) 
   return parameterizedHough:getHoughCorners(img_src)[{{}, {1,2}}], parameterizedHough:getHoughCorners(img_dest)[{{}, {1,2}}]
 end
 
 function Corners:getGeneralCorners(img_src, img_dest)
 	return parameterizedGeneral:getPoints(img_src), parameterizedGeneral:getPoints(img_dest)   
 end

 function Cornesr:getHoughCorners2(img_sc, img_dest)
 	return parameterizedHough:getHoughLinesAndPoints(img_src), parameterizedHough:getHoughLinesAndPoints(img_dest)
 end