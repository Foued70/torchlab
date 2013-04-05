local exports = require "util.util"

exports.geom        = require "util.geom"
exports.intersect   = require "util.intersect"
exports.bihtree     = require "util.bihtree"
exports.interpolate = require "util.interpolate"
exports.projection  = require "util.projection"

-- classes
exports.Ray   = require "util.Ray"


function exports.run_tests()
   local util_test = require "util.test.util-test"
   util_test.all()

   local geom_test = require "util.test.geom-test"
   geom_test.all()

   local intersect_test = require "util.test.intersect-test"
   intersect_test.all()

   -- FIXME pose tests broken since code moving around 4/3/2013
   -- local pose_test = require "util.test.pose-test"
   -- pose_test.all()
end

return exports
