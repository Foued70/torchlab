opencv = {}

opencv.C         = util.ffi.load('libopencv')
opencv.Mat       = require './Mat'
opencv.Detector  = require './Detector'
opencv.Extractor = require './Extractor'
opencv.Matcher   = require './Matcher.lua'
opencv.ImgProc   = require './ImgProc'
return opencv
