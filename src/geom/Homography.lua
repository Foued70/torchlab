require 'math'

--Represents a 3x3 homography
Homography = Class()

function Homography:__init(arg1, arg2) 
   if not(arg2) then
      local tensorMatrix = arg1
      if tensorMatrix:size()[1] ~=3 or tensorMatrix:size()[2]~=3 then
         error("wrong input size, homography must be three by three")
      end
      self.H = tensorMatrix;
   else
      local angle = arg1
      local translation = arg2
      local cosA = math.cos(angle)
      local sinA = math.sin(angle)
      self.H = torch.Tensor({{cosA, sinA, translation[1]},{-sinA, cosA, translation[2]},{0, 0, 1}})
end
   -- origin starting point of the ray
end

function Homography:__write_keys()
  return {'H'}
end

--x and y are reversed in opencv
--return tensor equivalent of homography for opencv
function Homography:getEquivalentCV()
   --x,y is reversed in opencv!
   local bestT = self.H:clone()
   bestT[1][2] = -self.H[1][2]
   bestT[2][1]=-self.H[2][1]
   bestT[1][3]=self.H[2][3]
   bestT[2][3]=self.H[1][3]
   return bestT
end

function Homography:getEquivalentPCL()
   --x,y is reversed in opencv!
   local bestT = self.H:clone()
   bestT[1][2] = -self.H[1][2]
   bestT[2][1]=-self.H[2][1]
   return bestT
end

function Homography:getEquivalentPCLString()
   --x,y is reversed in opencv!
   local bestT = self:getEquivalentPCL()
   return string.format("%f,%f,%f,%f,%f,%f,%f,%f,%f", bestT[1][1], bestT[1][2], bestT[1][3],
         bestT[2][1], bestT[2][2], bestT[2][3],
         bestT[3][1], bestT[3][2], bestT[3][3])
end


--T is 3x3 transformation matrix, mat_src is 2xn matrix
function Homography:applyToPoints(mat_src)
   if (mat_src:size(1) == 2) then
      return torch.mm(self.H, torch.cat(mat_src, torch.ones(1,mat_src:size(2)),1))
   elseif (mat_src:size(1) == 3) then
      return torch.mm(self.H, mat_src)
   end
end

--T is 3x3 transformation matrix, mat_src is 2xn matrix
function Homography:applyToPointsReturn2d(mat_src)
   return self:applyToPoints(mat_src)[{{1,2},{}}]
end

--returns new Homography ( self.H*homography2.H)
function Homography:combineWith(homography2)
   return Homography.new(self.H*homography2.H)
end