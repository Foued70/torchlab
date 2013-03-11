local qt = require 'qt'
local paths = require 'paths'
local qtuiloader = require 'qtuiloader'


local function loadFolder() 
end

local pp = {}

function pp.start()
  widget = qtuiloader.load(paths.dirname(paths.thisfile())..'/pp.ui')
  qt.connect(btnImgFolder, 'clicked()', loadFolder)
  widget:show()
end

return pp