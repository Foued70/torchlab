require('qt')
require('qtuiloader')
local paths = require('paths')


local function loadFolder() 
end

local pp = {}

function pp.start()
  local widget = qtuiloader.load(paths.dirname(paths.thisfile())..'/pp.ui')
  qt.connect(widget.btnImgFolder, 'clicked()', loadFolder)
  widget:show()
end

return pp