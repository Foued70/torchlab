-- This script requires qt and thus native torch, not cloudlab's luvit+torch

require 'image'
require 'nn'

require 'qtuiloader'
require 'qtwidget'
require 'qt'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Make a seamless texture')
cmd:text()
cmd:text('Options')
cmd:option('-imageFile', '', 'image to crop and make seamless')
cmd:option('-repeatStyle', 'repeat', 'mirror,mirrorH,mirrorV,offset,repeat')
cmd:option('-scale', '1/4', 'scale at which to process the image')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)
imgfile = params.imageFile
scale   = tonumber(params.scale) or 1/4
rStyle  = params.repeatStyle

--load image
orig = image.load(imgfile)

orig_size = orig:size()
-- set scale
scale_size    = torch.LongStorage(orig_size:totable())
scale_size[2] = scale_size[2]*scale
scale_size[3] = scale_size[3]*scale

-- set tile
tile = {}

-- photoshop style offset
function offset (imgtile, ox, oy, output)
   output = output or torch.Tensor()
   output:resize(imgtile:size())

   output[{{},{1, oy},{1, ox}}]     =
      imgtile[{{},{-oy,-1},{-ox,-1}}]

   output[{{},{1, oy},{ox+1,-1}}]   =
      imgtile[{{},{-oy,-1},{1,-ox-1}}]

   output[{{},{oy+1,-1},{1, ox}}]   =
      imgtile[{{},{1,-oy-1},{-ox,-1}}]

   output[{{},{oy+1,-1},{ox+1,-1}}] =
      imgtile[{{},{1,-oy-1},{1,-ox-1}}]

   return output
end

function four_way (imgtile, output)

   output = output or torch.Tensor()

   local h = imgtile:size(2)
   local w = imgtile:size(3)
   output:resize(imgtile:size(1),2*h,2*w)

   -- mode mirror
   output[{{},  {1, h},  {1, w}}] = imgtile
   output[{{},  {1, h},{w+1,-1}}] = imgtile
   output[{{},{h+1,-1},  {1, w}}] = imgtile
   output[{{},{h+1,-1},{w+1,-1}}] = imgtile

   return output
end

-- kaliedoscope (works when the textures can be mirrored
function mirror_4way (imgtile, output)

   output = output or torch.Tensor()

   local h = imgtile:size(2)
   local w = imgtile:size(3)
   output:resize(imgtile:size(1),2*h,2*w)

   -- mode mirror
   local imgtileh   = image.hflip(imgtile)
   local imgtilehv  = image.vflip(imgtileh)
   local imgtilehvh = image.hflip(imgtilehv)
   output[{{},  {1, h},  {1, w}}] = imgtilehvh
   output[{{},  {1, h},{w+1,-1}}] = imgtilehv
   output[{{},{h+1,-1},  {1, w}}] = imgtile
   output[{{},{h+1,-1},{w+1,-1}}] = imgtileh

   return output
end

-- kaliedoscope (works when the textures can be mirrored
function mirror_2way_h (imgtile, output)

   output = output or torch.Tensor()

   local h = imgtile:size(2)
   local w = imgtile:size(3)
   output:resize(imgtile:size(1),2*h,2*w)

   -- mode mirror
   local imgtileh   = image.hflip(imgtile)
   output[{{},  {1, h},  {1, w}}] = imgtile
   output[{{},  {1, h},{w+1,-1}}] = imgtileh
   output[{{},{h+1,-1},  {1, w}}] = imgtile
   output[{{},{h+1,-1},{w+1,-1}}] = imgtileh

   return output
end

-- kaliedoscope (works when the textures can be mirrored
function mirror_2way_v (imgtile, output)

   output = output or torch.Tensor()

   local h = imgtile:size(2)
   local w = imgtile:size(3)
   output:resize(imgtile:size(1),2*h,2*w)

   -- mode mirror
   local imgtilev   = image.vflip(imgtile)
   output[{{},  {1, h},  {1, w}}] = imgtile
   output[{{},  {1, h},{w+1,-1}}] = imgtile
   output[{{},{h+1,-1},  {1, w}}] = imgtilev
   output[{{},{h+1,-1},{w+1,-1}}] = imgtilev

   return output
end

function process()


   if not scaled or (scale_size ~= scaled:size()) then
      scaled = image.scale(orig,scale_size[3],scale_size[2])
      imyuv  = image.rgb2yuv(scaled)
      donorm = true
   end

   tile.y = math.floor(1 + (widget.Yin.value * scale_size[2] / 100))
   tile.h = math.floor(1 + (widget.Yout.value * scale_size[2] / 100))

   tile.x = math.floor(1 + (widget.Xin.value * scale_size[3] / 100))
   tile.w = math.floor(1 + (widget.Xout.value * scale_size[3] / 100))

   -- local constrast normalisation
   if donorm or not nbhd or not ylcn or nbhd:size(1) ~= gs then
      gs = 9 -- math.floor((tile.w - tile.x) * normscale )
      -- gaussian kernel must be odd
      if gs %2 == 0 then gs = gs + 1 end

      nbhd = image.gaussian({size=gs,sigma=1.591/gs,normalize=true})
      ylcn = image.lcn(imyuv[1],nbhd)
      nh = ylcn:size(1)
      nw = ylcn:size(2)

      imgnrml = imyuv:narrow(2,imyuv:size(2)/2 - nh/2,nh):narrow(3,imyuv:size(3)/2 - nw/2,nw)
      imgnrml[1] = ylcn

      -- for display
      imgnrml = scaled -- image.yuv2rgb(imgnrml)
      
      outh = imgnrml:size(2)
      outw = imgnrml:size(3)

      yin  = math.max(1,math.min(tile.y, outh - 1))
      yout = math.max(yin + 1, math.min(tile.h,outh))

      xin  = math.max(1,math.min(tile.x, outw -1))
      xout = math.max(xin + 1, math.min(tile.w,outw))

      imgtile = imgnrml[{{},{yin,yout},{xin,xout}}]
      if rStyle == 'mirror' then
         mtile = mirror_4way(imgtile,mtile)
      elseif rStyle == 'offset' then 
         mtile = offset(imgtile,(yout-yin)/2, (xout-xin)/2,mtile)
      elseif rStyle == 'mirrorV' then 
         mtile = mirror_2way_v(imgtile,mtile)
      elseif rStyle == 'mirrorH' then 
         mtile = mirror_2way_h(imgtile,mtile)
      else
         -- repeat
         mtile = four_way(imgtile,mtile)
      end

      tile.y = yin
      tile.h = yout
      tile.x = xin
      tile.w = xout

   end
end

-- display function
function display()

   win:gbegin()
   win:showpage()
   img2offset = scaled:size(2) + 1 
   -- (1) display input image
   image.display{image=scaled, win=win}
   image.display{image=mtile, win=win , y = img2offset}

   -- (2) overlay bounding boxes tile chosen
   win:setcolor(1,0,0)
   win:rectangle(tile.x, tile.y, tile.w -tile.x, tile.h - tile.y)
   win:stroke()
   win:setfont(qt.QFont{serif=false,italic=false,size=16})
   win:moveto(tile.x, tile.y-1)
   win:show('input')

   win:setcolor(0,1,0)
   outOffsetX = mtile:size(3)/4
   outOffsetY = mtile:size(2)/4 + img2offset

   win:rectangle(outOffsetX, outOffsetY,
                 mtile:size(3)/2, mtile:size(2)/2)
   win:stroke()
   win:setfont(qt.QFont{serif=false,italic=false,size=16})
   win:moveto(outOffsetX, outOffsetY-3)
   win:show('output')

   win:gend()
end

-- setup GUI (external UI file)
if not win or not widget then
   widget = qtuiloader.load('seamless_g.ui')
   win = qt.QtLuaPainter(widget.frame)
end

-- setup gui
timer = qt.QTimer()
timer.interval = 1
timer.singleShot = true
qt.connect(timer,
           'timeout()',
           function()
              process()
              display()
              timer:start()
           end)

widget.windowTitle = 'Seamless texture maker'
widget:show()
timer:start()
