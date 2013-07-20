local libflann  = require '../kdtree/libflann'
local ffi    = require 'ffi'
local ctorch = util.ctorch

local kdtree = Class()