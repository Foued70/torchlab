opencv = {}

opencv.C         = util.ffi.load('libopencv')

-- Classes use Mat.new(), Detector.new() etc. 
opencv.Mat       = require './Mat'
opencv.Detector  = require './Detector'
opencv.Extractor = require './Extractor'
opencv.Matcher   = require './Matcher'

-- groups of functions 
opencv.imgproc   = require './imgproc'
opencv.calib3d   = require './calib3d'
opencv.utils     = require './utils'

return opencv
