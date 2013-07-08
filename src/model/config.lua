local config = {}

model.config=Class()

--config.sweep_folder_prefix = "sweep_"
--config.img_folder = 'JPG'
--config.img_extensions = {'.jpg', '.png'}
--config.output_folder = 'output' -- folder name for new obj, mtl and processed textures
--config.delayed_start_rotation = -0.77 * math.pi
--config.rig_offset_position = {-0.18, -0.1, 0.2}
--config.rig_offset_rotation = {0.0123, -0.0123, 0.7070, 0.7070} -- 2 degrees around +x, 90 degrees around +z axis

--return config

sweep_folder_prefix = "sweep_"
img_folder = 'JPG'
img_extensions = {'.jpg', '.png'}
output_folder = 'output' -- folder name for new obj, mtl and processed textures
delayed_start_rotation = -0.77 * math.pi
rig_offset_position = {-0.18, -0.1, 0.2}
rig_offset_rotation = {0.0123, -0.0123, 0.7070, 0.7070} -- 2 degrees around +x, 90 degrees around +z axis
