require 'torch'
require 'dok'
require 'paths'

local exports = require "util/local"

exports.geom = require "util/geom"

exports.pbx  = require "util/pbx"
exports.obj  = require "util/obj"

-- classes
exports.Ray   = require "util/Ray"
exports.Poses = require "util/Poses"

require "util/global"

function exports.run_tests()
   local util_test = require "util/test/util-test"
   util_test.all()

   local geom_test = require "util/test/geom-test"
   geom_test.all()

   local pose_test = require "util/test/pose-test"
   pose_test.all()
end

return exports
