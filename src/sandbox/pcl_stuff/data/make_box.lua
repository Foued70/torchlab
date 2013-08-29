fs = require 'fs'

fname = "box.pcd"

fd = fs.openSync(fname,'w','0644')

xrange = torch.linspace(-1,1,30)
x = xrange:reshape(1,30):expand(30,30):reshape(30*30)

yrange = -torch.linspace(-1,1,30)
y = yrange:reshape(30,1):expand(30,30):reshape(30*30)

npts = y:size(1)

totpts = npts * 6

fs.writeSync(fd,-1, [[# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1 
]] ..
"WIDTH " .. totpts ..
[[

HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
]] ..
"POINTS " .. totpts ..
[[

DATA ascii
]])

for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",x[i],y[i],1))
end
for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",x[i],1,y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",1,x[i],y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",x[i],y[i],-1))
end
for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",x[i],-1,y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(fd,-1,string.format("%f %f %f\n",-1,x[i],y[i]))
end

fs.closeSync(fd)
