opencv = {}

opencv.C         = util.ffi.load('libopencv')
opencv.Mat       = require './Mat'
opencv.Detector  = require './Detector'
opencv.Extractor = require './Extractor'

return opencv
