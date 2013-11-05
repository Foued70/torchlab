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

   self.ops = nn.SpatialMaxPooling(self.win_width,self.win_height,1,1)
end

function NMS:forward(input)
   -- update to 1 x H x W ...
   if input:nDimension() == 2 then
      input:resize(util.util.add_slices(1,input:size()))
   end
   self.ops:forward(input)
   local maxout     = self.ops.output
   maxout = maxout[{{},{1,maxout:size(2)-1},{1,maxout:size(3)-1}}]
   local out_height = self.off_height + maxout:size(2)-1
   local out_width  = self.off_width  + maxout:size(3)-1
   
   self.output = self.output or torch.Tensor(input:size())
   self.output:resize(input:size()):zero()

   local ivals = 
      input[{{},{self.off_height,out_height},{self.off_width,out_width}}]

   local outnarrow  = 
      self.output[{{},{self.off_height,out_height},{self.off_width,out_width}}]

   outnarrow:eq(ivals,maxout)
   outnarrow:cmul(ivals)

   return self.output:squeeze()
   
end
