require 'luarocks.loader'
require 'torch'
require 'dok'
require 'xlua'

p = xlua.print
log = require 'util.log'

CLOUDLAB_SRC = paths.dirname(debug.getinfo(1).short_src)
