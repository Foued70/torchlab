ctorch = util.ctorch
ffi = require 'ffi'
log = require '../util/log'
Mat = opencv.Mat

ffi.cdef [[
]]

libstitcher =  util.ffi.load("libstitcher")

