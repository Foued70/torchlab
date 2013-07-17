local NMS = Class()
-- non maximal suppression using the nn SpatialMax module

-- Usage;

-- nms = image.NonMaximalSuppression.new()
-- i = image.load('test.jpg')
-- s = image.saliency.high_entropy_features(i)
-- n = nms:forward(s)
-- i[1][n:gt(0)] = 1 -- color maximal pixels red
-- image.save('test.png',i)

function NMS:__init(win_width, win_height)
   self.win_width  = win_width or 9
   self.win_height = win_height or 9

   -- kernel must be odd for this to make sense
   if self.win_width % 2 == 0 then 
      self.win_width = self.win_width + 1
   end
   if self.win_height % 2 == 0 then 
      self.win_height = self.win_height + 1
   end

   self.off_width  = math.ceil(self.win_width/2)
   self.off_height = math.ceil(self.win_height/2)

   self.ops = nn.Sequential()
   self.ops:add(nn.SpatialMaxPooling(self.win_width,self.win_height,1,1))
end

function NMS:forward(input)
   -- update to 1 x H x W ...
   if input:nDimension() == 2 then
      input:resize(util.util.add_slices(1,input:size()))
   end
   self.ops:forward(input)
   local maxout = self.ops.output
   self.output = self.output or torch.Tensor(input:size())
   self.output:resize(input:size()):zero()

   local ivals = input:narrow(2,self.off_height,maxout:size(2)):narrow(3,self.off_width,maxout:size(3))
   local outnarrow = self.output:narrow(2,self.off_height,maxout:size(2)):narrow(3,self.off_width,maxout:size(3))
   outnarrow:eq(ivals,maxout)
   outnarrow:cmul(ivals)
   
   return self.output
   
end
