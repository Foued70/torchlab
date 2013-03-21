require 'torch'
require 'xlua'
require 'image'

local function collect_garbage_for_period(duration_seconds)
  p(collectgarbage("count")/1024)
  local start_time = sys.clock()
  local end_time = start_time + duration_seconds
  while sys.clock() < end_time do
    collectgarbage("step", 0)
  end
  p(collectgarbage("count")/1024)
end


op = xlua.OptionParser('pose_correspondence.lua [options]')
op:option{'-d', '--dir', action='store', dest='dir', help='directory to load images from', req=true}
--op:option{'-p', '--pos', action='store' dest = 'pos', help='table holding pose of sweep', req=true}
opt = op:parse()
op:summarize()

local files = {}
for file in paths.files(opt.dir) do
  local path = paths.concat(opt.dir,file)
  if path:sub(#path-3,#path-3) == '.' then
    table.insert(files, path)	
  end
end

if #files == 0 then
   error('The directory specified doesnt contain any files')
end

table.sort(files, function (a,b) return a < b end)

print('Found files:')
p(files)
collectgarbage()
--collect_garbage_for_period(1.0)

local images = {}
for i,file in ipairs(files) do
   i = image.load(file)
   i = i:transpose(2,3)
   i = image.vflip(i)
   i = i:contiguous()
   table.insert(images, i)
   collectgarbage()
end

print('Loaded images:')
p(images)

glw = require('ui2.GLWidget').new()
while glw.initialized ~= true do
  os.execute("sleep " .. tonumber(1))
end

local interior_data = require('util.obj2').new('../ui2/objs/dryDragon.obj')
local interior_object = glw.renderer:add_object(interior_data)

local max_size = 800
local image_width 	= images[1]:size()[3]
local image_height 	= images[1]:size()[2]
--GL can only render images so big. Scale down if necessary
if (image_width > max_size) or (image_height > max_size) then
	if image_width > image_height then
		image_height = (image_height * max_size) / image_width
		image_width = max_size
	else
		image_width = (image_width * max_size) / image_height
    image_height = max_size
  end
end

--TODO: get monitor resolution. Fit to monitor while keeping aspect ratio of image
require('libui2').make_current(glw.qt_widget)
glw.renderer:create_camera('pose_camera', math.floor(image_width), math.floor(image_height))
glw.renderer:activate_camera('pose_camera')

--glw.renderer.cameras.pose_camera.eye[1] = opt.pos[1]
--glw.renderer.cameras.pose_camera.eye[2] = opt.pos[2]
--glw.renderer.cameras.pose_camera.eye[3] = opt.pos[3]
--TODO: convert pose data quaternion to a vector from eye +90(dslr is rotated 90 from matterport)
--local matterport_quat = torch.Tensor({opt.pos[4], opt.pos[5], opt.pos[6], opt.pos[7]})
--glw.renderer.cameras.pose_camera.center = eye + rotate(quat_to_vector(matterport_quat), 90)

glw.renderer:create_scene('wireframe_scene')
glw.renderer:activate_scene('wireframe_scene')
glw.renderer:create_camera('wireframe_camera', math.floor(image_width), math.floor(image_height))
glw.renderer:activate_camera('wireframe_camera')

local billboard_data = require('util.obj2').new('../ui2/objs/planeNormalized.obj')
local billboard_object = glw.renderer:add_object(billboard_data)
local wireframe_mat_data = {name='wireframe_mat', ambient={0,0,0,1}, diffuse={0,0,0,1}, specular={0,0,0,1}, shininess={0,0,0,1}, emission={0,0,0,1}}
local wireframe_mat = glw.renderer:create_material(wireframe_mat_data, glw.renderer.shaders.wireframe, {'pose_camera_frame_buffer_pass_picking'})
billboard_object.mesh:override_materials(wireframe_mat)

local vertex_highlight_mat = glw.renderer:create_material(wireframe_mat_data, glw.renderer.shaders.vertex_highlight, {'pose_camera_frame_buffer_pass_depth'})

glw.renderer:create_camera('vertex_highlight_camera', math.floor(image_width), math.floor(image_height))

--Standard Render of Model
glw.renderer:activate_scene('viewport_scene')
glw.renderer:activate_camera('pose_camera')
glw.renderer:render()

--Wireframe Render of Model
glw.renderer:activate_scene('wireframe_scene')
glw.renderer:activate_camera('wireframe_camera')
glw.renderer:render()

--Vertex Highlighting Render of Model
glw.renderer:activate_camera('vertex_highlight_camera')
glw.renderer:activate_scene('viewport_scene')
interior_object.mesh:override_materials(vertex_highlight_mat)
glw.renderer:render()

--Convert Passes to QImages
local wireframe_image_tensor = glw.renderer.cameras.wireframe_camera.frame_buffer:get_color_image()
local wireframe_image_qt = qt.QImage.fromTensor(wireframe_image_tensor)

local vertex_highlight_image_tensor = glw.renderer.cameras.vertex_highlight_camera.frame_buffer:get_color_image()
local vertex_highlight_image_qt = qt.QImage.fromTensor(vertex_highlight_image_tensor)

local dslr_image_qt = qt.QImage.fromTensor(images[1])

--[[
--Save QImages as files
depth_image_qt:save('depth.png', 'png')
p("saved depth.png")

wireframe_image_qt:save('wireframe.png', 'png')
p("saved wireframe.png")

vertex_highlight_image_qt:save('vertex_highlight.png', 'png')
p("saved vertex_highlight.png")
]]--

local pose_refinement_ui = require('PoseRefinementUi').new('pose_refinement.ui')
pose_refinement_ui:attach_image(1, 'dslr_photo', 'SourceOver', dslr_image_qt)
pose_refinement_ui:attach_image(2, 'wireframe', 'SourceOver', wireframe_image_qt)
pose_refinement_ui:attach_image(3, 'vertices', 'SourceOver', vertex_highlight_image_qt)

pose_refinement_ui:print_layer_info()

p("Done!")
