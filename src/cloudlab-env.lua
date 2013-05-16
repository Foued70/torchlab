require 'luarocks.loader'
require 'torch'
require 'dok'
require 'xlua'

p = xlua.print
log = require 'util.log'

CLOUDLAB_SRC = paths.dirname(debug.getinfo(1).short_src)
HOME = os.getenv('HOME')

require 'util.Class'

function printf (...)
   print(string.format(...))
end

local shell = require('shell.shell')
_G.shell_evaluate = shell.evaluate
_G.shell_is_complete = shell.is_complete
