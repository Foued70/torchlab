ffi          = require 'ffi'
pcl_ffi  = require './pcl_ffi'

PCLPointCloud = Class()

local function destructor ()
   return function (pc)
      pcl_ffi.PointCloud_destroy(pc)
   end
end

local function destructorN ()
   return function (pc)
      pcl_ffi.PointCloudN_destroy(pc)
   end
end

local function destructorMesh ()
   return function (pc)
      pcl_ffi.PointCloudMesh_destroy(pc)
   end
end

--either filename or xyz and rgb tensors
--[[
examples:
pcl_ffi  = require './pcl_ffi'
ffi          = require 'ffi'

pcl_1 = pcl.PCLPointCloud.new("/Users/stavbraun/code/backup/src/sandbox/pcl_stuff/pcl_register/two_registrations/yay.pcd")


vec = torch.range(1,3*100):reshape(100,3)
--vec = torch.cat(torch.range(1,100):reshape(1,100),torch.zeros(2,100),1):t():contiguous()
vec2 = torch.ones(100,3)*13
pcl_1 = pcl.PCLPointCloud.new(vec, vec2)
pcl_1:toTensor()

vec3 = torch.range(2,3*100+1):reshape(100,3)
vec4 = torch.ones(100,3)
pcl_2 = pcl.PCLPointCloud.new(vec3, vec4)

t,conv = pcl.PCLPointCloud.doICP(pcl_2, pcl_1, nil, nil,nil, 2))
]]--
function PCLPointCloud:__init(name, rgb)
   if(type(name) == "string") then
      self.cloud = ffi.gc(pcl_ffi.PointCloud_fromFile(ffi.string(name)), destructor())
   elseif (type(name) == "cdata") then
      self.cloud = name
   elseif (type(name) == "userdata" and type(rgb) == "userdata") then
      if name:size(1) ~= rgb:size(1) then
         error("two tensors should be same size")
      elseif name:size(2)~=3 or rgb:size(2)~=3 then
         erorr("both xyz and rgb size should be n by 3")
      end
      local name = name:double()
      local rgb = rgb:double()
      name:contiguous()
      rgb:contiguous()
      self.cloud = ffi.gc(pcl_ffi.PointCloud_fromTensor(torch.cdata(name), torch.cdata(rgb)), destructor())
   end
end


function PCLPointCloud:toTensor(output_xyz, output_rgb)
   local output_xyz = output_xyz or torch.DoubleTensor()
   local output_rgb = output_rgb or torch.DoubleTensor()

   pcl_ffi.PointCloud_toTensor(self.cloud, torch.cdata(output_xyz), torch.cdata(output_rgb))
   return output_xyz, output_rgb
end

function PCLPointCloud:toPCDFile(fname)
   if(type(fname)~="string") then
      error("filename not string")
   end

   pcl_ffi.PointCloud_toFile(self.cloud, ffi.string(fname))
end

function PCLPointCloud.fromPoints(points, rgb)
   pcNew = PCLPointCloud.new(points, rgb)
   return pcNew
end


function PCLPointCloud.fromPointCloud(pc, useNormals)
   if ((pc.__classname__ ~= "PointCloud/PointCloud")) then
      error("not the correct input type, should be PointCloud")
   end
   pcNew = PCLPointCloud.new(pc.points, pc.rgb)
   if(useNormals) then
      local idx, mask = pc:get_index_and_mask()
      local normals = pc:get_normal_map()
      local ns1 = normals:size(2)
      local ns2 = normals:size(3)
      normals = normals:reshape(3,ns1*ns2):t()  
      mask = (mask:reshape(1,ns1*ns2):squeeze()*-1+1):byte()
      local normals_n = torch.zeros(torch.gt(mask,0):sum(),3)
      normals_n[{{},1}] = normals[{{},1}][mask]
      normals_n[{{},2}] = normals[{{},2}][mask]
      normals_n[{{},3}] = normals[{{},3}][mask]
      pcNew:setNormalFromTensor(normals_n)
   end
   return pcNew
end

function PCLPointCloud:uniformSample(scale)
   local scale = scale or .01
   local pc = ffi.gc(pcl_ffi.PointCloud_uniformSample(self.cloud, scale), destructor())
   return PCLPointCloud.new(pc)
end

function PCLPointCloud:getNormalsAsTensor(output_normals)
   if(not(self.norm)) then
      error("need to set norms first")
   end
   local output_normals = output_normals or torch.DoubleTensor()
   pcl_ffi.PointCloud_normalToTensor(self.norm, torch.cdata(output_normals))
   return (output_normals)
end

function PCLPointCloud:calculateNormals(ksearch)
   local ksearch = ksearch or 10
   self.norm = ffi.gc(pcl_ffi.PointCloud_getNormals(self.cloud, ksearch), destructorN())
   return self.norm
end

function PCLPointCloud:setNormalFromTensor(normal)
   self.norm = ffi.gc(pcl_ffi.PointCloud_getNormals_fromTensor(torch.cdata(normal)), destructorN())
   return self.norm
end

function PCLPointCloud:growRegions(minClusterSize, numNeighbors)
   local minClusterSize = 100
   local numNeighbors = 10
   local smoothnessThreshold = 7*math.pi/180
   local curvatureThreshold = curvatureThreshold or .75
   if not(self.norm) then
      error("need to set norms first")
   end
   local indices = torch.LongTensor()
   local loc_change = torch.LongTensor()
   local output_color = pcl_ffi.growRegions(self.cloud, self.norm, torch.cdata(indices), torch.cdata(loc_change), minClusterSize, 
      numNeighbors, smoothnessThreshold, curvatureThreshold)
   output_color = ffi.gc(output_color, destructor())
   local cluster_sizes =  (loc_change[{{2,loc_change:size(1)}}]-loc_change[{{1,loc_change:size(1)-1}}])
   local sorted, ordering = torch.sort(cluster_sizes,true)
   local largest_cluster = {}
   local pcl_cluster = {}
   local orig_xyz, orig_color = self:toTensor()
   local orig_normals = self:getNormalsAsTensor()

   --save 10 largest clusters
   for i =1,10 do
      local lower_place = loc_change[ordering[i]]
      local higher_place = loc_change[ordering[i]+1]
      largest_cluster[i] = indices:index(1,torch.range(lower_place, higher_place):long())
      
      local pcNew = PCLPointCloud.new(orig_xyz:index(1,largest_cluster[i]), orig_color:index(1,largest_cluster[i]))
      pcNew:setNormalFromTensor(orig_normals:index(1,largest_cluster[i]))
      pcl_cluster[i] = pcNew
   end

   return PCLPointCloud.new(output_color), pcl_cluster, indices, loc_change
end

function PCLPointCloud:mesh()
   radiusSearch = 1
   mu = 10
   maximumNearestNeighbors = 500
   maxSurfaceAngle = math.pi/5
   minAngle = 30*math.pi/180
   maxAngle = 120*math.pi/180
   local output_mesh = ffi.gc(pcl_ffi.greedy_proj_mesh(self.cloud, self.norm, 
      mu, radiusSearch, maximumNearestNeighbors, maxSurfaceAngle, minAngle, maxAngle), destructorMesh());
end
function PCLPointCloud.getEuclideanValidationScore(pc1, pc2, max_range)
   if ((pc1.__classname__ ~= "pcl/PCLPointCloud") or (pc2.__classname__ ~= "pcl/PCLPointCloud")) then
      error("not the correct input type, should be PCLPointCloud")
   end
   local max_range = max_range or .3
   score = pcl_ffi.PointCloud_getEuclideanValidationScore(pc1.cloud, pc2.cloud, max_range)
return score
end

function PCLPointCloud.doICP(pc1, pc2, epsilon, maxIterations, ransacIterations, maxCorrespondDis, transf)
   if ((pc1.__classname__ ~= "pcl/PCLPointCloud") or (pc2.__classname__ ~= "pcl/PCLPointCloud")) then
      error("not the correct input type, should be PCLPointCloud")
   end
   local epsilon = epsilon or 1e-6
   local maxIterations = maxIterations or 100
   local ransacIterations=ransacIterations or 10000
   local maxCorrespondDis=maxCorrespondDis or .2
   local transf = transf or torch.DoubleTensor()
   hasConverged= pcl_ffi.PointCloud_doICP(pc1.cloud, pc2.cloud, torch.cdata(transf), epsilon, maxIterations, ransacIterations, maxCorrespondDis)
   return transf:clone(), hasConverged
end


function PCLPointCloud.alignNew(pc1, pc2, transf, corr) --.3 unif/.6 worked well
   error("not yet tested")
   descr_radius = .12
   cg_size = 5
   cg_thresh = .01 
   original_downsample = .01
   keypoint_downsample = .05
   pc1_unif = pc1:uniformSample(original_downsample)
   pc2_unif = pc1:uniformSample(original_downsample)
   pc1_unifN = pc1_unif:getNormals()
   pc2_unifN = pc2_unif:getNormals()

   pc1_down = pc1_unif:uniformSample(keypoint_downsample)
   pc2_down = pc2:uniformSample(keypoint_downsample)
   pc1_descriptors = pcl_ffi.PointCloud_computeDescriptors(pc1_down, pc1_unif, pc1_unifN, descr_radius);
   pc2_descriptors = pcl_ffi.PointCloud_computeDescriptors(pc2_down, pc2_unif, pc2_unifN, descr_radius);
   
   transf = transf or torch.DoubleTensor()
   corr = corr or torch.DoubleTensor()

   pcl_ffi.PointCloud_computeTransformation(pc1_descriptors, pc2_descriptors, pc1_down, pc2_down, 
      transf, corr, cg_size, cg_thresh);

   return transf, corr
end
