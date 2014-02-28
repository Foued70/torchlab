local ffi    = require 'ffi'

ffi.cdef
[[
	int get_normal_map(double* normal_map, double* xyz_map, char* valid_mask, int height, int width);
]]

local libpvm   = util.ffi.load('libpvm')

self = {}

function self.get_normals( xyz_map, valid_mask)
	normal_map = torch.Tensor(xyz_map:size())
	libpvm.get_normal_map(torch.data(normal_map), torch.data(xyz_map), torch.data(valid_mask), xyz_map:size(2), xyz_map:size(3))
	return normal_map
end

return self