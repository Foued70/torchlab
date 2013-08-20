opencv = {}

opencv.C         = util.ffi.load('libopencv')
opencv.Mat       = require './Mat'
opencv.Detector  = require './Detector'
opencv.Extractor = require './Extractor'
opencv.Matcher   = require './Matcher'
opencv.ImgProc   = require './ImgProc'
return opencv
