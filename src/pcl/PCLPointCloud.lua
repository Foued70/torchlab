ffi          = require 'ffi'
pcl_ffi  = require './pcl_ffi'

PCLPointCloud = Class()

local function destructor ()
   return function (pc)
      pcl_ffi.PointCloud_destroy(pc)
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
pcl_1 = pcl.PCLPointCloud.new(vec)
pcl_1:toTensor()

vec3 = torch.range(2,3*100+1):reshape(100,3)
pcl_2 = pcl.PCLPointCloud.new(vec3)

t,conv = pcl.PCLPointCloud.doICP(pcl_2, pcl_1, nil, nil,nil, 2)
]]--
function PCLPointCloud:__init(name)
   if(type(name) == "string") then
      self.cloud = ffi.gc(pcl_ffi.PointCloud_fromFile(ffi.string(name)), destructor())
   elseif (type(name) == "cdata") then
      self.cloud = name
   elseif (type(name) == "userdata") then
      if name:size(2)~=3 then
         erorr("xyz size should be n by 3")
      end
      local name = name:double()
      name = name:contiguous()
      self.cloud = ffi.gc(pcl_ffi.PointCloud_fromTensor(torch.cdata(name)), destructor())
   end
end


function PCLPointCloud:toTensor(output_xyz)
   local output_xyz = output_xyz or torch.DoubleTensor()

   pcl_ffi.PointCloud_toTensor(self.cloud, torch.cdata(output_xyz))
   return output_xyz
end

function PCLPointCloud:toPCDFile(fname)
   if(type(fname)~="string") then
      error("filename not string")
   end

   pcl_ffi.PointCloud_toFile(self.cloud, ffi.string(fname))
end

function PCLPointCloud.fromPoints(points)
   pcNew = PCLPointCloud.new(points:contiguous())
   return pcNew
end


function PCLPointCloud.fromPointCloud(pc)
   if ((pc.__classname__ ~= "PointCloud/PointCloud")) then
      error("not the correct input type, should be PointCloud")
   end
   pcNew = PCLPointCloud.new(pc.points)
   return pcNew
end

function PCLPointCloud:uniformSample(scale)
   local scale = scale or .01
   local pc = ffi.gc(pcl_ffi.PointCloud_uniformSample(self.cloud, scale), destructor())
   return PCLPointCloud.new(pc)
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
