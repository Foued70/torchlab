cplane_ffi = require('./cplane_ffi')

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
scan_num = 3
extract_planes_test( scan_num )
