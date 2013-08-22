--Represents a 3x3 homography
Class()
require './Homography'
Homography = geom.Homography

--in the transformation we are trying to solve for t b=At, where A is dependent on source points, t is the transformation we are solving for 
-- we will returned the reshaped matrix such that T*x represents where the point x gets mapped
--goal is to solve:
   --[d0_x d1_x; d0y d1y; 1 1] = [t0 -t1 t2; t1 t0 t3; 0 0 1]*[s0x s1x; s0y s1y; 1 1]
   --or equivalently
   --[d0_x; d0_y; d1_x; d1_y] = [s0_x s0_y 1 0; -s0_y s0_x 0 1; s0x -s1y 1 0; s1y s1x 0 1] * [t_0; t_1; t_2; t_3]
   --left is destination matrix ([d0_x; d0_y; d1_x; d1_y] ), A is source matrix ([-s0y...])
function getMatrixFromSourcePoints(src_pt1, src_pt2) 
   return torch.Tensor({{src_pt1[1], -src_pt1[2], 1, 0},{src_pt1[2], src_pt1[1], 0, 1},{src_pt2[1],-src_pt2[2], 1,0}, {src_pt2[2], src_pt2[1], 0, 1}})
end

function getMatrixFromDestination(dest_pt1, dest_pt2)
   return torch.Tensor({{dest_pt1[1]}, {dest_pt1[2]}, {dest_pt2[1]}, {dest_pt2[2]}})
end

function getInverseMatrixFromSource(src_pt1, src_pt2) 
   return torch.inverse(getMatrixFromSourcePoints(src_pt1, src_pt2))
end

--get the actual transformation matrix T such that T*src points = dest points
function getHomography(A_inv, b) 
   t = torch.mm(A_inv, b):resize(4)
   return Homography.new(torch.Tensor({{t[1], -t[2], t[3]}, {t[2], t[1], t[4]}, {0, 0, 1}} ))
end