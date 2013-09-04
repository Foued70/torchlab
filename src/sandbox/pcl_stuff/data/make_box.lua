fs = require 'fs'

pcdfname = "box.pcd"
xyzfname = "box.xyz"
dorange = false

pcdfd = fs.openSync(pcdfname,'w','0644')
xyzfd = fs.openSync(xyzfname,'w','0644')

-- range
if dorange then

   xrange = torch.linspace(-1,1,30)
   x = xrange:reshape(1,30):expand(30,30):reshape(30*30)

   yrange = -torch.linspace(-1,1,30)
   y = yrange:reshape(30,1):expand(30,30):reshape(30*30)

else
   x = (torch.rand(900) * 2) -1
   y = (torch.rand(900) * 2) -1 
end

npts = y:size(1)
totpts = npts * 6

fs.writeSync(pcdfd,-1, 
             [[# .PCD v0.7 - Point Cloud Data file format
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
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",x[i],y[i],1))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",x[i],y[i],1))
end
for i = 1,x:size(1) do
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",x[i],1,y[i]))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",x[i],1,y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",1,x[i],y[i]))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",1,x[i],y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",x[i],y[i],-1))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",x[i],y[i],-1))
end
for i = 1,x:size(1) do
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",x[i],-1,y[i]))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",x[i],-1,y[i]))
end
for i = 1,x:size(1) do
   fs.writeSync(pcdfd,-1,string.format("%f %f %f\n",-1,x[i],y[i]))
   fs.writeSync(xyzfd,-1,string.format("%f %f %f\n",-1,x[i],y[i]))
end

fs.closeSync(pcdfd)
fs.closeSync(xyzfd)
