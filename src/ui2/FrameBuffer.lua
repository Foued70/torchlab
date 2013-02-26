local gl = require 'ui2.gl'
local libui = require 'libui2'

local FrameBuffer = torch.class('FrameBuffer')


function FrameBuffer:__init(width, height)
  self.width = width
  self.height = height
  self.frame_buffer_id = gl.GenFramebuffer()
  self:bind()

  self.color_tex_id = self:generate_texture(gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, gl.COLOR_ATTACHMENT0)
  self.picking_tex_id = self:generate_texture(gl.RGB32UI, gl.RGB_INTEGER, gl.UNSIGNED_INT, gl.COLOR_ATTACHMENT1)
  self.depth_tex_id = self:generate_texture(gl.DEPTH_COMPONENT32, gl.DEPTH_COMPONENT, gl.FLOAT, gl.DEPTH_ATTACHMENT)
  gl.CheckFramebuffer()

  self.attachments = gl.enum(2)
  self.attachments[0] = gl.COLOR_ATTACHMENT0
  self.attachments[1] = gl.COLOR_ATTACHMENT1
end

function FrameBuffer:__gc()
  gl.DeleteTexture(self.depth_tex_id)
  gl.DeleteTexture(self.picking_tex_id)
  gl.DeleteTexture(self.depth_tex_id)
  gl.DeleteFrameBuffer(self.frame_buffer_id)
end


function FrameBuffer:generate_texture(pixel_format_internal, pixel_format, value_type, attachement)
  local id = gl.GenTexture()
  gl.BindTexture(gl.TEXTURE_2D, id)

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)

  gl.TexImage2D(gl.TEXTURE_2D, 0, pixel_format_internal, self.width, self.height, 0, pixel_format, value_type, nil) -- Can I put nil here?
  gl.BindTexture(gl.TEXTURE_2D, 0)

  gl.FramebufferTexture2D( gl.FRAMEBUFFER, attachement, gl.TEXTURE_2D, id, 0)

  return id
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
  log.trace(depth[0])
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


