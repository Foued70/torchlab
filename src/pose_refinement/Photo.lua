require 'torch'
require 'image'

local util = require "util" -- need printf
local LensSensor = require "util.LensSensor"
local projection = require "util.projection"

local Photo = torch.class('Photo')

local REQUIRED_CALIBRATION_PAIRS = 3

function Photo:__init(image_path)
  self.calibration_pairs = torch.Tensor(REQUIRED_CALIBRATION_PAIRS,5):fill(0)
  self.pairs_calibrated = 0
  self.vertex_set = false
  self.image_coordinate_set = false
  self.white_wall = false

  self.image_path = image_path
  self.image_data = nil

  self.lens = nil
  self.rectilinear_lut = nil
end

function Photo:add_vertex(vertex_position)
  if self.pairs_calibrated < REQUIRED_CALIBRATION_PAIRS then
    local pair_index = self.pairs_calibrated + 1
    self.calibration_pairs[pair_index]:sub(1, 3):copy(vertex_position)
    self.vertex_set = true
    self:update_calibration_status()
  end
end

function Photo:add_image_coordinate(screen_position)
  if self.pairs_calibrated < REQUIRED_CALIBRATION_PAIRS then
    local pair_index = self.pairs_calibrated + 1
    self.calibration_pairs[pair_index]:sub(4, 5):copy(screen_position)
    self.image_coordinate_set = true
    self:update_calibration_status()
  end
end

function Photo:update_calibration_status()
  if (self.vertex_set == true) and (self.image_coordinate_set == true) then
    self.pairs_calibrated = self.pairs_calibrated + 1
    self.vertex_set = false
    self.image_coordinate_set = false
  end
end

function Photo:calibration_complete()
  return (self.pairs_calibrated == REQUIRED_CALIBRATION_PAIRS)
end

function Photo:image_loaded()
  return not (self.image_data == nil)
end

function Photo:load_image()
  log.trace("Loading image from path:", "\""..self.image_path.."\"", "at:", sys.clock())
  local image_native = image.load(self.image_path)

  --TODO: Move Lens out of here to proper place in sweep
  self.lens = LensSensor.new("nikon_D5100_w10p5mm",image_native)
  self.rectilinear_lut = self.lens:to_projection("rectilinear")
  self.image_data = projection.remap(image_native,self.rectilinear_lut)
  lens = nil
  rectilinear_map = nil
  collectgarbage()
  log.trace("Completed image load at:", sys.clock())
end

function Photo:flush_image()
  image_data = nil
  collectgarbage()
end

return Photo