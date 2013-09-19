
local path = require 'path'

Icp = Class()
Icp.sweep_directory = ""
Icp.transform_executable = "~/cloudlab/bin/pcl_transform"


function Icp.alignAndICP(sweep_name1, sweep_name2, transformation, cloud_output_name, transform_output_name, 
	icpcloud_output_name, icptransform_output_name, zshift, properties)

print(Icp.transform_executable .. 
   		" --source ".. sweep_name1 ..
   		" --destination " ..  sweep_name2  ..  
   		" --transformation " .. transformation:getEquivalentPCLString() .. 
   		" --output_preicp " ..  cloud_output_name .. 
   		" --output_preicp_transform " .. transform_output_name ..
   		" --output_posticp " ..  icpcloud_output_name .. 
   		" --output_posticp_transform " .. icptransform_output_name ..
   		" --scale " .. properties.scale ..
   		" --icp_correspondence " .. properties.icp_correspondence ..
   		" --icp_max_iterations " .. properties.icp_max_iterations ..
   		" --icp_ransac_iterations " .. properties.icp_ransac_iterations ..
   		" --icp_transform_eps " .. properties.icp_transform_eps ..
         " --zshift " .. zshift)

   print(util.fs.exec(Icp.transform_executable .. 
         " --source ".. sweep_name1 ..
         " --destination " ..  sweep_name2  ..  
         " --transformation " .. transformation:getEquivalentPCLString() .. 
         " --output_preicp " ..  cloud_output_name .. 
         " --output_preicp_transform " .. transform_output_name ..
         " --output_posticp " ..  icpcloud_output_name .. 
         " --output_posticp_transform " .. icptransform_output_name ..
         " --scale " .. properties.scale ..
         " --icp_correspondence " .. properties.icp_correspondence ..
         " --icp_max_iterations " .. properties.icp_max_iterations ..
         " --icp_ransac_iterations " .. properties.icp_ransac_iterations ..
         " --icp_transform_eps " .. properties.icp_transform_eps ..
         " --zshift " .. zshift))

end