require 'torch'
require 'image'

local util = require "util" -- need printf
local LensSensor = require "util.LensSensor"
local projection = require "util.projection"

local Photo = torch.class('Photo')

local REQUIRED_CALIBRATION_PAIRS = 3

function Photo:__init(parent_sweep, image_path)
  self.sweep = parent_sweep
  self.calibration_pairs = torch.Tensor(REQUIRED_CALIBRATION_PAIRS,5):fill(0)
  self.pairs_calibrated = 0
  self.vertex_set = false
  self.image_coordinate_set = false
  self.white_wall = false

  self.image_path = image_path
  self.image_data_raw = nil
  self.image_data_rectilinear = nil

  self.lens = nil
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
  return (self.image_data_raw ~= nil) and (self.image_data_rectilinear ~= nil)
end

function Photo:load_image()
  log.trace("Loading image from path:", "\""..self.image_path.."\"", "at:", sys.clock())
  self.image_data_raw = image.load(self.image_path)
  self.lens = self.sweep.scan:get_lens(self.image_data_raw)
  self.image_data_rectilinear = projection.remap(self.image_data_raw, self.lens.rectilinear)
  log.trace("Completed image load at:", sys.clock())
end

function Photo:flush_image()
  self.lens = nil
  self.image_data_raw = nil
  self.image_data_rectilinear = nil
  collectgarbage()
end

return Photo