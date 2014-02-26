
ArcIO = data.ArcIO
test_glfw = require('./test_glfw')

job_id = 'precise-transit-6548' 
work_id = 'test_glfw'
scan_num = 6

arc_io = ArcIO.new( job_id, work_id )
pc = arc_io:getScan( scan_num )

xyz_map = pc:get_xyz_map()

print(xyz_map:size())
print(xyz_map)

ArcIO = data.ArcIO

test_glfw.draw_rays(torch.cdata(xyz_map))
