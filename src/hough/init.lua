local pi = math.pi
local twopi = 2*pi

function line (img,numR,numA)
   local transf = torch.Tensor(numR,numA)
   -- expects a 2D input 
   if (img:dim == 3) then 
      img = img:sum(1):squeeze()
   end

   local width  = img:size(2)
   local height = img:size(1)
   local half_width  = width*0.5
   local half_height = height*0.5
   

   local A = torch.linspace(0,twopi,numA)

   local sinA = torch.sin(A)
   local cosA = torch.cos(A)
   
   local x = torch.linspace(-half_width,half_width,width)
   local y = torch.linspace(-half_height,half_height,height)
   
   local xcosA = torch.zeros(x:size(1),cosA:size(1))
   -- outer product
   xcosA:addr(x,cosA)

   local ysinA = torch.zeros(y:size(1),sinA:size(1))
   ysinA:addr(y,sinA)

   local r = xcosA:repeatTensor(360,1,1)
   
   r:add(ysinA:transpose(2,1,3))

end