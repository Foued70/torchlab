require 'qt'
require 'qtgui'

local qtuiloader = require('qtuiloader')
local paths = require('paths')
local fs = require('util/fs')

local Pose = torch.class('Pose') -- simplified Pose class, maybe should use the Pose in util?
local Sweep = torch.class('Sweep')
local Pic = torch.class('Pic')
local PosePicker = torch.class('PosePicker')

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

function Sweep:__init(folder)  
  self.path = folder
  self.name = paths.basename(folder)
  self.pics = {}
  
  for i, f in ipairs(fs.files_only(folder, ".jpg", ".png")) do
    table.insert(self.pics, Pic.new(f))
  end
end
  
function Pic:__init(filePath)
  self.path = filePath
  self.name = paths.basename(filePath)
end

function PosePicker:__init()  
  self.poses = {}
  self:resetSweeps()
  
  self.ui = qtuiloader.load(paths.dirname(paths.thisfile())..'/pp.ui')  
  qt.connect(self.ui.btnPicsFolder, 'clicked()', function() self.loadFolder(self) end)
  qt.connect(self.ui.btnPoseFile, 'clicked()', function() self.loadPoseFile(self) end)
  qt.connect(self.ui.btnNext, 'clicked()', function() self.next(self) end)
  qt.connect(self.ui.btnPrev, 'clicked()', function() self.prev(self) end)
  self.ui:show()
end

function PosePicker:loadFolder() 
  self.picsFolder = qt.QFileDialog.getExistingDirectory(self.ui, "Select Folder"):tostring()
  
  if not self.picsFolder or string.len(self.picsFolder) == 0 or not paths.dirp(self.picsFolder) then 
    self.ui.labelPicsFolder:setText("could not load folder")
    return 
  end
  
  self.ui.labelPicsFolder:setText(paths.basename(self.picsFolder))
  self:resetSweeps()
  
  local picsDirs = fs.dirs_only(self.picsFolder)
  if picsDirs and #picsDirs > 0 then
    for i, v in ipairs(picsDirs) do
      table.insert(self.sweeps, Sweep.new(v))
    end
  else
    table.insert(self.sweeps, Sweep.new(self.picsFolder))
  end
  self.lastSweepIdx = #self.sweeps
  self.lastPicIdx = #self.sweeps[self.lastSweepIdx].pics
  self:updateGui()
end

function PosePicker:loadPoseFile()
  local poseFile = qt.QFileDialog.getOpenFileName(self.ui, "Select Pose File", '', "Text Files (*.txt)"):tostring()  
  
  if not poseFile or string.len(poseFile) == 0 or not paths.filep(poseFile) then
    self.ui.labelPoseFile:setText("could not load pose file")
    return
  end
  
  -- guess the obj file based on the pose file
  local objFiles = fs.files_only(paths.dirname(poseFile), ".obj")
  if objFiles and #objFiles > 0 then self.objFile = objFiles[1] end  
  if not self.objFile then self.ui.labelPoseFile:setText("could not load obj file using pose path") return end
  
  self.ui.labelPoseFile:setText(paths.basename(poseFile))

  local f, err = io.open(poseFile, "r")
  if err then self.ui.labelPoseFile:setText(err) return end
  
  self.poses = {}
  local t = f:read("*all")
  for line in string.gmatch(t, "[^\r\n]+") do
    table.insert(self.poses, Pose.new(line))
  end
  f:close()
  
  self:updateGui()
end

function PosePicker:savePoseData()
  if not self.sweeps or #self.sweeps == 0 then p('no sweeps') return end
  if not paths.dirp(self.picsFolder) then p('invalid folder to write to') return end
  
  local f, err = io.open(self.picsFolder..'poses.txt', 'w')
  if err then p('error opening file', err) return end
    
  for i, sweep in ipairs(self.sweeps) do
    -- TODO: write the pic path and pic pose 
  end
  f:close()
end

function PosePicker:resetSweeps()
  self:savePoseData()
  self.sweeps = {}
  self.currSweepIdx = 1
  self.currPicIdx = 1
  self.lastSweepIdx = 1
  self.lastPicIdx = 1
end

function PosePicker:defaultPicPose(i)
  if #self.poses >= i then 
    return self.poses[i]
  else
    return self.poses[1]  
  end
end

function PosePicker:updateGui()
  -- enabled state of next and prev btns
  self.ui.btnNext:setDisabled(self.currSweepIdx == self.lastSweepIdx and self.currPicIdx == self.lastPicIdx)
  self.ui.btnPrev:setDisabled(self.currPicIdx == 1 and self.currSweepIdx == 1)
  
  -- set the currSweep and currPic vars
  self.currSweep = self.sweeps[self.currSweepIdx]
  if self.currSweep and #self.currSweep.pics > 0 then
    self.currPic = self.currSweep.pics[self.currPicIdx]
  end
  
  -- sweep and pic labels that indicate which sweep and which pic we're on  
  if self.currSweep then
    self.ui.labelCurrSweep:setText(string.format("%s %d/%d", self.currSweep.name, self.currSweepIdx, self.lastSweepIdx))  
    if self.currPic then
      self.ui.labelCurrPic:setText(string.format("%s %d/%d", self.currPic.name, self.currPicIdx, #self.currSweep.pics))
    end
  end

  -- update the orig pose label
  local defaultPose = self:defaultPicPose(self.currSweepIdx)
  if defaultPose then
    self.ui.poseOrigText:setText(self:formatPose(defaultPose))
  end
    
  -- update the gl widget sandwich with the current pic the pic's pose  
  self:updatePoseText()
end

function PosePicker:updatePoseText()
  if self.currPic and self.currPic.pose then
    self.ui.poseNewText:setText(self:formatPose(self.currPic.pose))    
  end
end

function PosePicker:formatPose(pose)  
  return string.format("Name: %s \nQuat: %f, %f, %f, %f \nXYZ: %f, %f, %f \nUV: %f, %f \nDegrees: %f, %f", pose.name, pose.quat[1][1], pose.quat[1][2], pose.quat[1][3], pose.quat[1][4], pose.xyz[1][1], pose.xyz[1][2], pose.xyz[1][3], pose.center_u[1], pose.center_v[1], pose.degree_per_px_x[1], pose.degree_per_px_y[1])
end

function PosePicker:calcPicPose()
  -- TODO: pass a table of correspondences to method in __ (?) and set currPic.pose to returned Pose table
  
  -- update the pose of pics in the current sweep that are after the current pic if they don't have a pose yet
  self:propagatePicPose()
  self:updatePoseText()
end


function PosePicker:propagatePicPose()
  -- this hasn't been tested...
  if not self.currPic and self.currPic.pose then return end
  
  -- only adjusts the pose of pics in the currSweep but could also adjust later sweep's pics' pose 
  -- as a mod of the default pose from poseFile
  for i=self.currPicIdx, #self.currSweep.pics, 1 do
    local pic = self.currSweep.pics[i]
    if not pic.pose then pic.pose = self.currPic.pose end
  end
end

function PosePicker:next()
  if self.currPicIdx == #self.currSweep.pics then
    self.currSweepIdx = self.currSweepIdx+1
    self.currPicIdx = 1
  else
    self.currPicIdx = self.currPicIdx+1
  end
  
  self:updateGui()
end

function PosePicker:prev()
  if self.currPicIdx == 1 then
    self.currSweepIdx = self.currSweepIdx-1
    self.currPicIdx = #self.currSweep.pics
  else
    self.currPicIdx = self.currPicIdx-1
  end
  self:updateGui()
end


return PosePicker