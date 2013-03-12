local gl = require 'ui2.gl'
local libui = require 'libui2'
local Material = require 'ui2.Material'

local FrameBuffer = torch.class('FrameBuffer')

function FrameBuffer:__init(widget, name, width, height)
  self.widget = widget
  self.name = name
  self.width = width
  self.height = height
  self.frame_buffer_id = gl.GenFramebuffer()
  self:bind()

  local pass_name_color = self.name..'_pass_color'
  self.widget.texture_manager:create_blank(pass_name_color, gl.RGBA, self.width, self.height, gl.RGBA, gl.UNSIGNED_BYTE)
  self:attach_texture(gl.COLOR_ATTACHMENT0, self.widget.texture_manager.textures[pass_name_color])

  local pass_name_picking = self.name..'_pass_picking'
  self.widget.texture_manager:create_blank(pass_name_picking, gl.RGB32UI, self.width, self.height, gl.RGB_INTEGER, gl.UNSIGNED_INT)
  self:attach_texture(gl.COLOR_ATTACHMENT1, self.widget.texture_manager.textures[pass_name_picking])

  local pass_name_depth = self.name..'_pass_depth'
  self.widget.texture_manager:create_blank(pass_name_depth, gl.DEPTH_COMPONENT32, self.width, self.height, gl.DEPTH_COMPONENT, gl.FLOAT)
  self:attach_texture(gl.DEPTH_ATTACHMENT, self.widget.texture_manager.textures[pass_name_depth])

  gl.CheckFramebuffer()

  self.attachments = gl.enum(2)
  self.attachments[0] = gl.COLOR_ATTACHMENT0
  self.attachments[1] = gl.COLOR_ATTACHMENT1

  log.trace("FrameBuffer Constructed.")
end

function FrameBuffer:__gc()
  self.widget.texture_manager:delete_texture(self.name..'_pass_color')
  self.widget.texture_manager:delete_texture(self.name..'_pass_picking')
  self.widget.texture_manager:delete_texture(self.name..'_pass_depth')
  gl.DeleteFramebuffer(self.frame_buffer_id)
end

function FrameBuffer:attach_texture(attachment, id)
  gl.FramebufferTexture2D( gl.FRAMEBUFFER, attachment, gl.TEXTURE_2D, id, 0)
end

function FrameBuffer:use()
  self:bind()
  gl.DrawBuffers(2, self.attachments)
end

function FrameBuffer:display()
  gl.BindFramebuffer(gl.DRAW_FRAMEBUFFER, 0)

  gl.BindFramebuffer(gl.READ_FRAMEBUFFER, self.frame_buffer_id)
  gl.ReadBuffer(gl.COLOR_ATTACHMENT0)
  gl.check_errors()


  gl.BlitFramebuffer(
    0, 0, self.width, self.height,
    0, 0, self.width, self.height,
    gl.COLOR_BUFFER_BIT,
    gl.LINEAR)

  gl.check_errors()

  self.unbind() -- TODO: are we even bound here? 
                -- We are. 
                -- before unbinding, our draw buffer is set to 0 (default framebuffer)
                -- before unbinding, our read buffer is set to self.frame_buffer_id
                -- after unbinding, both draw and read are set to 0. gl.FRAMEBUFFER means gl.DRAW_FRAMEBUFFER and gl.READ_FRAMEBUFFER
end


function FrameBuffer:read_depth_pixel(x, y)
  -- OpenGL saves its framebuffer pixels upsidedown. Need to flip y to get intended data.
  local read_x = math.floor(x)
  local read_y = math.floor(self.height-y)
  self:bind()
  gl.PixelStorei(gl.PACK_ALIGNMENT, 1)

  local depth = gl.float(1)
  gl.ReadPixels( read_x, read_y, 1, 1, gl.DEPTH_COMPONENT, gl.FLOAT, depth)
  
  gl.ReadBuffer(gl.NONE)
  self:unbind()
  return depth[0]
end


function FrameBuffer:read_pick_pixel(x, y)
  -- OpenGL saves its framebuffer pixels upsidedown. Need to flip y to get intended data.  
  local read_x = math.floor(x)
  local read_y = math.floor(self.height-y)
  self:bind()

  gl.ReadBuffer(gl.COLOR_ATTACHMENT1)
  gl.PixelStorei(gl.PACK_ALIGNMENT, 1)

  local pick_pixel = gl.uint(3)
  gl.ReadPixels(read_x, read_y, 1, 1, gl.RGB_INTEGER, gl.UNSIGNED_INT, pick_pixel)
  
  gl.ReadBuffer(gl.NONE)
  self:unbind()

  -- object_id, triangle_id = submesh_start + primitive_id
  return pick_pixel[0], pick_pixel[1] + pick_pixel[2]
end



function FrameBuffer:bind()
  gl.BindFramebuffer(gl.FRAMEBUFFER, self.frame_buffer_id)
end

function FrameBuffer:unbind()
  gl.BindFramebuffer(gl.FRAMEBUFFER, 0)
end

return FrameBuffer


