--[[
	Testing polyclipping code
]]--

polyclipping_ffi = require('./polyclipping_ffi')

mask = torch.IntTensor(400,400)
polyclipping_ffi.point_in_polygon(torch.cdata(mask))

image.display( mask:gt(0) )
