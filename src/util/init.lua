require 'torch'
require 'dok'
require 'paths'

local exports = require "util/util"

exports.geom      = require "util/geom"
exports.intersect = require "util/intersect"

exports.pbx  = require "util/pbx"
exports.obj  = require "util/obj"

-- classes
exports.Ray   = require "util/Ray"
exports.Pose  = require "util/Pose"
exports.Poses = require "util/Poses"

require "util/global"

function exports.run_tests()
   local util_test = require "util/test/util-test"
   util_test.all()

   local geom_test = require "util/test/geom-test"
   geom_test.all()

   local intersect_test = require "util/test/intersect-test"
   intersect_test.all()

   local pose_test = require "util/test/pose-test"
   pose_test.all()
end

return exports
