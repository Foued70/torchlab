local exports = require "util/util"

exports.geom        = require "util/geom"
exports.intersect   = require "util/intersect"
exports.bihtree     = require "util/bihtree"
exports.interpolate = require "util/interpolate"
exports.projection  = require "util/projection"

exports.depot = require "util/depot"
exports.obj   = require "util/obj"
exports.obj2  = require "util/obj2"

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
