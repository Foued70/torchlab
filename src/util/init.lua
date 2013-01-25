require 'torch'
require 'dok'
require 'paths'

local exports = {}

exports.pbx = require "util/pbx"
exports.geom = require "util/geom"
exports.obj = require "util/obj"
exports.pose = require "util/pose"

require "util/global"

function exports.run_tests()
   local geom_test = require "util/geom-test"
   geom_test.all()

   local pose_test = require "util/pose-test"
   pose_test.all()
end

return exports
