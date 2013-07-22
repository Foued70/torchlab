-- in this tutorial we walk through the different ways of writing a
-- function operating on an image in torch.
log.tic()
input = image.load("tmp.jpg","byte",128)
printf("load image in %2.4fs", log.toc() * 0.001)
print("input size",input:size())

R = input[1]
G = input[2]
B = input[3]
printf("R(%d,%d) G(%d,%d) B(%d,%d)",R:min(),R:max(),G:min(),G:max(),B:min(),B:max())

-- from http://www.fourcc.org/fccyvrgb.php
-- Y  =      (0.257 * R) + (0.504 * G) + (0.098 * B) + 16
-- Cb = U = -(0.148 * R) - (0.291 * G) + (0.439 * B) + 128
-- Cr = V =  (0.439 * R) - (0.368 * G) - (0.071 * B) + 128

output = input:clone()

Y = output[1]
U = output[2]
V = output[3]

log.tic()
for i = 1,input:size(2) do 
   for j = 1,input:size(3) do 
      Y[i][j] =  (R[i][j] * 0.257) + (G[i][j] * 0.504) + (B[i][j] * 0.098) + 16
      U[i][j] = -(R[i][j] * 0.148) - (G[i][j] * 0.291) + (B[i][j] * 0.439) + 128
      V[i][j] =  (R[i][j] * 0.439) - (G[i][j] * 0.368) - (B[i][j] * 0.071) + 128
   end
end
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

log.tic()
for i = 1,input:size(2) do 
   for j = 1,input:size(3) do 
      local Y = Y[i][j]
      local U = U[i][j]
      local V = V[i][j]
      local R = R[i][j]
      local G = G[i][j]
      local B = B[i][j]
      Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16 
      U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
      V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128 
   end
end
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())
print("what's going on?")

output = torch.Tensor(input:size())
Y = output[1]
U = output[2]
V = output[3]

log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())
print("what's going on? (2)")

inputDouble = torch.Tensor(input:size()):copy(input)
R = inputDouble[1]
G = inputDouble[2]
B = inputDouble[3]

log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())
print("Looks OK")


log.tic()
Y:copy(R):mul(0.257):add(0.504,G):add(0.098,B):add(16)
U:copy(R):mul(-0.148):add(-0.291,G):add(0.439,B):add(128)
V:copy(R):mul(0.439):add(-0.368,G):add(-0.071,B):add(128)
printf("process channels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

log.tic()
input = image.load("tmp.jpg","byte",128,"RGB","HWD")
printf("load image in %2.4fs", log.toc() * 0.001)
print("input size",input:size())

output = torch.Tensor(input:size())
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      output[i][j][1] =  (input[i][j][1] * 0.257) + (input[i][j][2] * 0.504) + (input[i][j][3] * 0.098) + 16
      output[i][j][2] = -(input[i][j][1] * 0.148) - (input[i][j][2] * 0.291) + (input[i][j][3] * 0.439) + 128
      output[i][j][3] =  (input[i][j][1] * 0.439) - (input[i][j][2] * 0.368) - (input[i][j][3] * 0.071) + 128
   end
end
printf("process pixels in %2.4fs", log.toc() * 0.001)
Y = output:select(3,1)
U = output:select(3,2)
V = output:select(3,3)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      local Y = output[i][j][1]
      local U = output[i][j][2]
      local V = output[i][j][3]
      local R = input[i][j][1]
      local G = input[i][j][2]
      local B = input[i][j][3]
      Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16 
      U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
      V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128 
   end
end
printf("process pixels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())


output = torch.Tensor(input:size())
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      output[{i,j,1}] =  (input[{i,j,1}] * 0.257) + (input[{i,j,2}] * 0.504) + (input[{i,j,3}] * 0.098) + 16
      output[{i,j,2}] = -(input[{i,j,1}] * 0.148) - (input[{i,j,2}] * 0.291) + (input[{i,j,3}] * 0.439) + 128
      output[{i,j,3}] =  (input[{i,j,1}] * 0.439) - (input[{i,j,2}] * 0.368) - (input[{i,j,3}] * 0.071) + 128
   end
end
printf("process pixels in %2.4fs", log.toc() * 0.001)
Y = output[{{},{},1}]
U = output[{{},{},2}]
V = output[{{},{},3}]
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      local Y = output[{i,j,1}]
      local U = output[{i,j,2}]
      local V = output[{i,j,3}]
      local R = input[{i,j,1}]
      local G = input[{i,j,2}]
      local B = input[{i,j,3}]
      Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16 
      U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
      V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128 
   end
end
printf("process pixels in %2.4fs", log.toc() * 0.001)
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

inputDouble = torch.Tensor(input:size()):copy(input)

-- explain :t()
mat = torch.Tensor({{ 0.257, 0.504, 0.098},
                    {-0.148,-0.291, 0.439},
                    {0.439,-0.368,-0.071}}):t()

v   = torch.Tensor({16,128,128})

-- Why not input:clone() ?
log.tic()
inputDouble = inputDouble:resize(input:size(1)*input:size(2),input:size(3))
printf("resize in %2.4fs", log.toc() * 0.001)

-- difference between resize and reshape
log.tic()
output = torch.mm(inputDouble,mat)
printf("mm in %2.4fs", log.toc() * 0.001)

log.tic()
v = v:reshape(1,3):expandAs(output)
printf("expand v in %2.4fs", log.toc() * 0.001)

log.tic()
output:add(v)
output:resize(input:size())
printf("add and resize in %2.4fs", log.toc() * 0.001)

print("output:size = ",output:size())
Y = output[{{},{},1}]
U = output[{{},{},2}]
V = output[{{},{},3}]
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",
       Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

print("all together 1")
log:tic()
output = torch.mm(inputDouble,mat):add(torch.Tensor({{16,128,128}}):expandAs(inputDouble)):resize(input:size())
printf("all together 1 in %2.4fs", log.toc() * 0.001)

Y = output[{{},{},1}]
U = output[{{},{},2}]
V = output[{{},{},3}]
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",
       Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())

print("all together 2")
log:tic()
output = torch.addmm(torch.Tensor({{16,128,128}}):expandAs(inputDouble),inputDouble,mat):resize(input:size())
printf("all together 2 in %2.4fs", log.toc() * 0.001)

Y = output[{{},{},1}]
U = output[{{},{},2}]
V = output[{{},{},3}]
printf("Y(%d,%d) U(%d,%d) V(%d,%d)",
       Y:min(),Y:max(),U:min(),U:max(),V:min(),V:max())
