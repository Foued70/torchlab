require 'torch'
require 'image'

local SweepCamera = torch.class('SweepCamera')

local REQUIRED_CALIBRATION_PAIRS = 3

function SweepCamera:__init(lens, offset_position, offset_rotation, image_path)
  self.lens = lens

  self.offset_position = offset_position
  self.offset_rotation = offset_rotation

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

function SweepCamera:load_image_from_file(image_path)
  local image_data = image.load(image_path)
  image_data = image_data:transpose(2,3)
  image_data = image.vflip(image_data)
  image_data = image_data:contiguous()
  collectgarbage()
  return image_data
end

function SweepCamera:flush_image()
  image_data = nil
  collectgarbage()
end

return SweepCamera