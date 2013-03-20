require 'torch'
require 'image'

local util = require "util" -- need printf
local LensSensor = require "util.LensSensor"
local projection = require "util.projection"

local SweepCamera = torch.class('SweepCamera')

local REQUIRED_CALIBRATION_PAIRS = 3

function SweepCamera:__init(image_path)

  self.calibration_pairs = torch.Tensor(REQUIRED_CALIBRATION_PAIRS,5)
  self.pairs_calibrated = 0
  self.vertex_set = false
  self.image_coordinate_set = false
  self.white_wall = false

  self.image_path = image_path
  self.image_data = nil
end

function SweepCamera:add_vertex(x, y, z)
  local pair_index = self.pairs_calibrated + 1

  self.calibration_pairs[pair_index][1] = x
  self.calibration_pairs[pair_index][2] = y
  self.calibration_pairs[pair_index][3] = z

  self.vertex_set = true
  self:update_calibration_status()
end

function SweepCamera:add_image_coordinate(u, v)
  local pair_index = self.pairs_calibrated + 1

  self.calibration_pairs[pair_index][4] = u
  self.calibration_pairs[pair_index][5] = v

  self.image_coordinate_set = true
  self:update_calibration_status()
end

function SweepCamera:update_calibration_status()
  if (self.vertex_set == true) and (self.image_coordinate_set == true) then
    self.pairs_calibrated = self.pairs_calibrated + 1
    self.vertex_set = false
    self.image_coordinate_set = false
  end
end

function SweepCamera:calibration_complete()
  return (self.pairs_calibrated == REQUIRED_CALIBRATION_PAIRS)
end

function SweepCamera:image_loaded()
  return not (self.image_data == nil)
end

function SweepCamera:load_image()
  log.trace("Loading image from path:", "\""..self.image_path.."\"", "at:", sys.clock())
  local image_native = image.load(self.image_path)
  
  --TODO:Get Lens Projections Working!
  --[[
  local lens = LensSensor.new("nikon_D5100_w10p5mm",image_native)
  local rectilinear_map = lens:to_projection("rectilinear")
  self.image_data = projection.remap(image_native,rectilinear_map)
  ]]--

  self.image_data = image_native
  self.image_data = self.image_data:transpose(2,3)
  self.image_data = image.vflip(self.image_data)
  self.image_data = self.image_data:contiguous()
  lens = nil
  rectilinear_map = nil
  collectgarbage()
  log.trace("Completed image load at:", sys.clock())
end

function SweepCamera:flush_image()
  image_data = nil
  collectgarbage()
end

return SweepCamera