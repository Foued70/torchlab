require '../image/init'
pcl = PointCloud.pointcloud

col = torch.Tensor({{1.0,0.0,0.0},
					{0.0,1.0,0.0},
					{0.0,0.0,1.0},
					{0.5,0.0,0.0},
					{0.0,0.5,0.0},
					{0.0,0.0,0.5},
					{0.25,0.1,0.0},
					{0.0,0.25,0.1},
					{0.1,0.0,0.25},
					{0.05,0.1,0.05}
					})

for i=73,74 do
if not (i == 91) then
range = 10
fname = '/Users/lihui815/cloudlab/src/data/test/faro/2013_07_17_scans/clean_'..i..'.xyz'
print('loading '..i)
a = pcl.new(fname,range)
print(a.radius)
print(a.count)
print('flattening')
a:make_flattened_images(0.01)
print('saving image')
image.save('/Users/lihui815/cloudlab/src/data/test/faro/a/a'..i..'.png',a.imagez)
a:write('/Users/lihui815/cloudlab/src/data/test/faro/xyz/ds'..i..'.xyz')

print('computing and saving hough transform')
tns = a.imagez[1]
hgh = image.hough.get_hough_transform(tns, math.max(tns:size(1), tns:size(2)),360*2)
image.save('/Users/lihui815/cloudlab/src/data/test/faro/h/h'..i..'.png',hgh)
lcn=image.hough.local_contrast_normalization(hgh)
image.save('/Users/lihui815/cloudlab/src/data/test/faro/c/c'..i..'.png',lcn)
print('computing best 8 lines')
fbl = image.hough.find_best_lines(lcn,8)
print(fbl)
print('drawing best 8 lines and saving image')
dwl = tns:clone()
for j=1,8 do
dwl = image.hough.draw_line(dwl,fbl[j][1],fbl[j][2],hgh:size(1),hgh:size(2),col[j][1],col[j][2],col[j][3])
image.save('/Users/lihui815/cloudlab/src/data/test/faro/d/d'..i..'.png',dwl)
end

a=nil
tns = nil
hgh=nil
lcn=nil
fbl=nil
dwl=nil

print('done')
end
collectgarbage()
end