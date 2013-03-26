local config = {}

config.sweep_folder_prefix = "sweep_"
config.img_folder = 'JPG'
config.img_extensions = {'.jpg', '.png'}
config.output_folder = 'output' -- folder name for new obj, mtl and processed textures
config.delayed_start_rotation = -0.8 * math.pi
config.rig_offset_position = {0, -0.1, 0.3}
config.rig_offset_rotation = {0, 0, 0.7071, 0.7071} -- 90 degrees around +z axis
--config.rig_offset_rotation = {0, 0, 0, 1} -- 90 degrees around +z axis

return config