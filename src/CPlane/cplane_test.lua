cplane_ffi = require('./cplane_ffi')
specs = data.ArcSpecs
--[[
test = CPlane.CPlane.test_cplane
test('precise-transit-6548', 10)
]]--

--[[
extract_planes_test = CPlane.CPlane.extract_planes_test
scan_num = 10
extract_planes_test( scan_num )
]]--

extract_planes_test = CPlane.CPlane.extract_planes_random


scan_ids = specs.scan_ids
for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]
	extract_planes_test( scan_num )
end
