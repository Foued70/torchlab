require 'qt'
require 'qtgui'
require 'torch'

local qtuiloader = require('qtuiloader')
local paths = require('paths')

local Pose = torch.class('Pose') -- simplified Pose, maybe should use the Pose in util?

function Pose:__init(poseStr)
  local poseVals = torch.Tensor(1, 11)
  local k = 1
  local firstVal = true
  for val in poseStr:gmatch("%S+") do    
    if firstVal then
      self.name = val
      firstVal = false      
    else      
      poseVals[1][k] = tonumber(val)
      k = k+1
    end
  end
  
  self.quat = poseVals:narrow(2,1,4)
  self.xyz = poseVals:narrow(2,5,3)
  self.center_u = poseVals:select(2,8)
  self.center_v = poseVals:select(2,9) 
  self.degree_per_px_x = poseVals:select(2,10)
  self.degree_per_px_y = poseVals:select(2,11)
end

local Sweep = torch.class('Sweep')

function Sweep:__init(folder)
  self.path = folder
  self.name = paths.basename(folder)
  self.pics = {}
  for f in paths.files(folder) do
    table.insert(self.pics, Pic.new(folder..f))
  end
end

local Pic = torch.class('Pic')
  
function Pic:__init(file)
  self.path = file
  self.name = paths.basename(file)
end

local PosePicker = torch.class('PosePicker')
function PosePicker:__init()
  self.sweeps = {}
  self.poses = {}
  
  self.ui = qtuiloader.load(paths.dirname(paths.thisfile())..'/pp.ui')  
  qt.connect(self.ui.btnPicsFolder, 'clicked()', function() self.loadFolder(self) end)
  qt.connect(self.ui.btnPoseFile, 'clicked()', function() self.loadPoseFile(self) end)
  qt.connect(self.ui.btnNext, 'clicked()', function() self.next(self) end)
  qt.connect(self.ui.btnPrev, 'clicked()', function() self.prev(self) end)
  self.ui:show()
end

function PosePicker:loadFolder() 
  self.picsFolder = qt.QFileDialog.getExistingDirectory(self.ui, "Select Folder"):tostring()

  if not self.picsFolder or string.len(self.picsFolder) == 0 then return end
  self.ui.labelPicsFolder:setText(self.picsFolder)  
  self:savePoseData()
  self.sweeps = {} 
end

function PosePicker:loadPoseFile()
  local poseFile = qt.QFileDialog.getOpenFileName(self.ui, "Select Pose File", '', "Text Files (*.txt)")  
  self.ui.labelPoseFile:setText(poseFile)

  poseFile = poseFile:tostring()  
  if not poseFile or string.len(poseFile) == 0 then return end

  local f, err = io.open(poseFile, "r")
  if err then self.ui.labelPoseFile:setText(err) return end
  
  self.poses = {}
  local t = f:read("*all")
  for line in string.gmatch(t, "[^\r\n]+") do
    table.insert(self.poses, Pose.new(line))
  end
  f:close()
end

function PosePicker:savePoseData()
  if #self.sweeps == 0 then p('no sweeps') return end
  if not paths.dirp(self.picsFolder) then p('invalid folder to write to') return end
  
  local f, err = io.open(self.picsFolder..'poses.txt', 'w')
  if err then p('error opening file', err) return end
    
  for i, sweep in ipairs(self.sweeps) do
    -- write the pic path and pic pose 
  end
  f:close()
end

function PosePicker:next()
  p('next')
end

function PosePicker:prev()
  p('prev')
end


return PosePicker