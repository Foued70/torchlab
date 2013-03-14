require('qt')
require ('qtgui')
local qtuiloader = require('qtuiloader')
local paths = require('paths')

local PosePicker = torch.class('PosePicker')
local Pose = torch.class('Pose') -- simplified Pose class, maybe should use util/Pose?

function Pose:__init(poseStr)
  
end

function PosePicker:__init()
  self.ui = qtuiloader.load(paths.dirname(paths.thisfile())..'/pp.ui')  
  qt.connect(self.ui.btnImgFolder, 'clicked()', function() self.loadFolder(self) end)
  qt.connect(self.ui.btnPoseFile, 'clicked()', function() self.loadPoseFile(self) end)
  qt.connect(self.ui.btnNext, 'clicked()', function() self.next(self) end)
  qt.connect(self.ui.btnPrev, 'clicked()', function() self.prev(self) end)
  self.ui:show()
end

function PosePicker:loadFolder() 
  local folderName = qt.QFileDialog.getExistingDirectory(self.ui, "Select Folder")
  self.ui.labelImgFolder:setText(folderName)
end

function PosePicker:loadPoseFile()
  local poseFile = qt.QFileDialog.getOpenFileName(self.ui, "Select Pose File", '', "Text Files (*.txt)")
  --if not poseFile or string.len(poseFile) == 0 then return end
  --qt.QMessageBox.information(self.ui, "Unable to open file", "Unable to open file")
  
  self.ui.labelPoseFile:setText(poseFile)
  self.poses = {}
  
end

function PosePicker:next()
  p('next')
end

function PosePicker:prev()
  p('prev')
end


return PosePicker