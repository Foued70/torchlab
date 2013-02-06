require 'torch'
require 'libui'

local ui = {}

function ui.display(obj)
   libui.display(obj)
end


function ui.create_object(obj)
   libui.create_object(obj)
end


return ui
