local NMS = Class()

require 'nn'

function NMS:__init(width, height)
   self.width  = width
   self.height = height

   -- kernel must be odd for this to make sense
   if self.width % 2 == 0 then 
      self.width = self.width + 1
   end
   if self.height % 2 == 0 then 
      self.height = self.height + 1
   end

   self.offW = math.ceil(self.width/2)
   self.offH = math.ceil(self.height/2)

   self.ops = nn.Sequential()
   self.ops:add(nn.SpatialMaxPooling(self.width,self.height,1,1))
end

function NMS:forward(input)
   -- update to 1 x H x W ...
   self.ops:forward(input)
   local maxout = self.ops.output
   self.output = self.output or torch.Tensor(input:size())
   self.output:resize(input:size()):zero()

   local ivals = input:narrow(2,self.offH,maxout:size(2)):narrow(3,self.offW,maxout:size(3))
   local outnarrow = self.output:narrow(2,self.offH,maxout:size(2)):narrow(3,self.offW,maxout:size(3))
   outnarrow:eq(ivals,maxout)
   outnarrow:cmul(ivals)
   
   return self.output
   
end
