cplane_ffi = require('./cplane_ffi')
specs = data.ArcSpecs

--test = CPlane.CPlane.test_classification
--[[
test = CPlane.CPlane.test_bilateral_smoothing
test()
]]--

--[[
extract_planes_test = CPlane.CPlane.extract_planes_test
scan_num = 10
extract_planes_test( scan_num )
]]--


--job_id = 'precise-transit-6548'


test = CPlane.CPlane.extract_planes_ransac
job_id = 'mobile-void-0590'

scan_ids = specs.scan_ids[job_id]
for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]
	test(job_id, scan_num)
end

