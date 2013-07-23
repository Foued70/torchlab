require '../image/init'
pcl = PointCloud.pointcloud

for i=70,94 do
if not (i == 91) then
range = 15
thresh = 1
fname = '/Users/lihui815/Projects/PointCloud/Scans/2013_07_17_scans/clean_'..i..'.xyz'
print('loading '..i)
a = pcl.new(fname,range)
print(a.maxval-a.minval)
print('downampling')
b = a:downsample(0.01, thresh)
print(a.count)
print(b.count)
print('flattening')
a:make_flattened_images(0.01)
b:make_flattened_images(0.01)
print('saving image')
image.save('/Users/lihui815/cloudlab/src/data/test/faro/a'..i..'.jpg',a.imagez)
image.save('/Users/lihui815/cloudlab/src/data/test/faro/b'..i..'.jpg',b.imagez)
b:write('/Users/lihui815/cloudlab/src/data/test/faro/ds'..i..'.xyz')
a=nil
b=nil
collectgarbage()
print('done')
end
end