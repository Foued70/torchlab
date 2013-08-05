slide = tutorial.util.print_slide()
next = tutorial.util.next
eval = tutorial.util.eval

slide()

print [[In this tutorial we walk through the different ways of writing a
function operating on an image in torch. 

Image operations are in the image package which is loaded
automatically on startup (cloudlab's package is different from
torch's).
]]

eval("image")
next(slide)

eval [[
log.tic()
input = image.load("img/lena.png","byte",128)
]]
printf("load image in %2.4fs", log.toc() * 0.001)
eval("input:size()")

next(slide) 

print [[ torch's images are stored channel x height x width, sometimes
refered to as 'DHW' 
]]

eval [[
R = input[1]
G = input[2]
B = input[3]
]]

eval("R:min(),R:max()")
eval("G:min(),G:max()")
eval("B:min(),B:max()")

next(slide)

print [[ In this tutorial we'll try to implement color conversion
algorithm from RGB to YUV (from http://www.fourcc.org/fccyvrgb.php):

   Y  =      (0.257 * R) + (0.504 * G) + (0.098 * B) + 16
   Cb = U = -(0.148 * R) - (0.291 * G) + (0.439 * B) + 128
   Cr = V =  (0.439 * R) - (0.368 * G) - (0.071 * B) + 128

First we make a new tensor to contain the output:
]]

eval("output = input:clone()")

print [[ Then we prepare and run our code.
]]

eval[[
Y = output[1]
U = output[2]
V = output[3]

log.tic()
for i = 1,input:size(2) do 
  for j = 1,input:size(3) do 
     Y[i][j]= (R[i][j]*0.257)+(G[i][j]*0.504)+(B[i][j]*0.098)+16
     U[i][j]=-(R[i][j]*0.148)-(G[i][j]*0.291)+(B[i][j]*0.439)+128
     V[i][j]= (R[i][j]*0.439)-(G[i][j]*0.368)-(B[i][j]*0.071)+128
  end
end 
]]

printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print [[ ok it works but 2.5s to process a small image? I thought torch was fast...
]]
next(slide)

print [[ Each of the lookups takes computation so perhaps we can do
better with fewer indexing operations.
]]

eval [[
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
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

print [[ It is a bit suprising that we get such a speed up with this
simple change.  Any thing in the inner loop gets run thousands to
millions of times. But this is pretty slow.  Is there anything else we
can do? ]]

next(slide)

print [[ Can we do without the loop ? The luajit interpreter is quite
fast but for loops in an interpreted language are slow. Vectors and
matrices have over ridden +,-,* and / operators.
]]

eval [[
log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print[[ Wow that is fast.  100x ! 
But what's going on with our min and max values? ]]
next(slide)

print [[ We loaded the image as a byte tensor. Makes sense for most
images, when we cloned the tensor to create the output tensor, it also
was a byte tensor so it can't collect the floating point operations
created in our math.
]]

eval("output:type()")
print [[ 
So we make the output a Double tensor. 
]]
eval("output = torch.Tensor(input:size())")
eval("output:type()")
eval [[
Y = output[1]
U = output[2]
V = output[3]
]]

eval [[
log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print("Still fast. But still bug. What's going on? (2)")
next(slide)

print [[ Each of the operations (R * 0.257) overrides the '*' operator
and calls the equivalent of "tmp1 = torch.mul(R,0.257)" which creates
a temporary tensor, but since our input is in Byte format, the whole
operation is in Byte.  0.257 in Byte is 0. So tmp1 is a Byte tensor of
zeros.  Same for all the other operations. Then +16, + 128 and thus
the min, max values.

How do we fix?
]]

eval("input:type()")
eval [[
inputDouble = torch.Tensor(input:size()):copy(input)
inputDouble:type()
]]
next(slide)
print [[ Now set up the computation again: 
]]
eval [[
R = inputDouble[1]
G = inputDouble[2]
B = inputDouble[3]
]]
print(" And compute ")
eval [[ 
log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print [[ Looks OK.  
A bit slower b/c we are operating in doubles.
Can we do better? ]]
next(slide)

print [[ Remember the tmp matrices which are created for each
operation when overriding the "*" operator ?  We can do without them
by specifying in which tensor we wish to accumulate the output:
]]

eval [[
log.tic()
Y:copy(R):mul(0.257):add(0.504,G):add(0.098,B):add(16)
U:copy(R):mul(-0.148):add(-0.291,G):add(0.439,B):add(128)
V:copy(R):mul(0.439):add(-0.368,G):add(-0.071,B):add(128)
]]

printf("process channels in %2.4fs", log.toc() * 0.001)

print [[ 
the :add() takes an optional second argument to make use of
the muladd which is one operation on modern cpus 
]]

eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print("Nice.  About 5x faster.")
next(slide)

print [[ What about the data layout. DHW that's weird right? many
languages use HWD with the channels in the last dimension.  This is a
deliberate decision by torch's creators.  All modern CPUs allow you to
pipeline operations, this requires that data be aligned in
memory. Allocating memory and moving data around is an expensive
operation. Let's see what happens when we do all the above operations
in HWD.  
]]

eval [[
log.tic()
input = image.load("img/lena.png","byte",128,"RGB","HWD")
input:size()
]]
printf("load image in %2.4fs", log.toc() * 0.001)
eval("output = input:clone()")
next(slide)
print [[ Now the original for loop: 
]]
eval [[
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      output[i][j][1]=(input[i][j][1]*0.257)+(input[i][j][2]*0.504)+(input[i][j][3]*0.098)+16
      output[i][j][2]=-(input[i][j][1]*0.148)-(input[i][j][2]*0.291)+(input[i][j][3] * 0.439) + 128
      output[i][j][3]=(input[i][j][1]*0.439)-(input[i][j][2]*0.368) - (input[i][j][3] * 0.071) + 128
   end
end
]]
printf("process pixels in %2.4fs", log.toc() * 0.001)
eval [[
Y = output:select(3,1)
U = output:select(3,2)
V = output:select(3,3)
]]
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

print [[ Slower, but kind of makes sense right ? There are 3 indexing
for each variable [i][j][1] and we already saw that the indexing was
slow. ]]

next(slide)

print [[ Can we fix the indexing ?: 
]]

eval [[ 
R = input[{{},{},1}] 
G = input[{{},{},2}]
B = input[{{},{},3}]

Y = output[{{},{},1}] 
U = output[{{},{},2}]
V = output[{{},{},3}]
]]

print [[ We can index the last channel (but it is non-contiguous in memory)
]]

eval [[
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
     Y[i][j]= (R[i][j]*0.257)+(G[i][j]*0.504)+(B[i][j]*0.098)+16
     U[i][j]=-(R[i][j]*0.148)-(G[i][j]*0.291)+(B[i][j]*0.439)+128
     V[i][j]= (R[i][j]*0.439)-(G[i][j]*0.368)-(B[i][j]*0.071)+128
   end
end
]]

print [[ Looks just like before.
]]
printf("process pixels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

print [[ Same amount of indexing, same speed. ]]
next(slide)

print [[ What about with less indexing, by storing the lookups in temp
variables. ]]

eval [[
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
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
]]
printf("process pixels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
next(slide)
print [[ Interesting note. Packing a sequence of indexing operations into a table is faster, as a single table gets passed to the C indexing code rather than 3 calls from the interpreter. 
]] 
eval [[
log.tic()
for i = 1,input:size(1) do 
   for j = 1,input:size(2) do 
      output[{i,j,1}] =  (input[{i,j,1}] * 0.257) + (input[{i,j,2}] * 0.504) + (input[{i,j,3}] * 0.098) + 16
      output[{i,j,2}] = -(input[{i,j,1}] * 0.148) - (input[{i,j,2}] * 0.291) + (input[{i,j,3}] * 0.439) + 128
      output[{i,j,3}] =  (input[{i,j,1}] * 0.439) - (input[{i,j,2}] * 0.368) - (input[{i,j,3}] * 0.071) + 128
   end
end
]]
printf("process pixels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

next(slide)
print [[ And when storing the temporary variables. ]]
eval [[
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
]]

printf("process pixels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

next(slide)
print [[ What about operations on the non-contiguous vectors ? ]]
eval [[
input = torch.Tensor(input:size()):copy(input)
R = input[{{},{},1}]
G = input[{{},{},2}]
B = input[{{},{},3}]
output = input:clone()
Y = output[{{},{},1}]
U = output[{{},{},2}]
V = output[{{},{},3}]
]]
eval [[ input:type() ]]
eval [[ output:type() ]]

next(slide)
eval [[
log.tic()
Y =  (R * 0.257) + (G * 0.504) + (B * 0.098) + 16
U = -(R * 0.148) - (G * 0.291) + (B * 0.439) + 128
V =  (R * 0.439) - (G * 0.368) - (B * 0.071) + 128
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print [[ About 3x slower ]]
next(slide)

eval [[
log.tic()
Y:copy(R):mul(0.257):add(0.504,G):add(0.098,B):add(16)
U:copy(R):mul(-0.148):add(-0.291,G):add(0.439,B):add(128)
V:copy(R):mul(0.439):add(-0.368,G):add(-0.071,B):add(128)
]]
printf("process channels in %2.4fs", log.toc() * 0.001)
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
print [[ About 3x slower ]]
next(slide)

print [[ Can we do something contiguous with the channels in the last
dimension ?  
]]

print [[ Use linear algebra. Store our operators in a matrix. 
]]
eval [[
mat = torch.Tensor({{ 0.257, 0.504, 0.098},
                    {-0.148,-0.291, 0.439},
                    {0.439,-0.368,-0.071}})

v   = torch.Tensor({16,128,128})
]]
print [[ Set up out input pixels as a matrix (nPixels x 3) :
]]

eval [[
log.tic()
input_size = input:size()
input = input:resize(input:size(1)*input:size(2),input:size(3))
]]
printf("resize in %2.4fs", log.toc() * 0.001)
next(slide)


-- difference between resize and reshape
print [[ Apply matrix multiplication. 
  (nPoint x nInputChannels) * (nInputChannels x  nOutputChannels)
]]
eval [[
log.tic()
output:resize(input:size())
torch.mm(output,input,mat:t())
v = v:reshape(1,3):expandAs(output)
output:add(v)
output:resize(input_size)
]]
printf("mm, expand, add, resize in %2.4fs", log.toc() * 0.001)

print("output:size = ",output:size())
eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]
next(slide)

print [[ We can do all the operations above in one line. ]]
eval [[
log:tic()
output:resize(input:size())
torch.mm(output,
         input,
         mat:t()):add(
  torch.Tensor({{16,128,128}}):expandAs(input)):resize(input_size)
]]
printf("all together in %2.4fs", log.toc() * 0.001)

eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

next(slide)

print [[ All together 2
]]
eval [[
log:tic()
torch.addmm(output,
            torch.Tensor({{16,128,128}}):expandAs(input),
            input,
            mat):resize(input_size)
]]
printf("all together 2 in %2.4fs", log.toc() * 0.001)

eval [[ Y:min(),Y:max() ]]
eval [[ U:min(),U:max() ]]
eval [[ V:min(),V:max() ]]

print [[ Still 3x slower than our best DHW ]] 

print [[ More tensor operations at http://www.torch.ch/manual/torch/maths ]]
