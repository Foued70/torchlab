--[[
	Bilateral Normal Smoothing
		I'm following: http://web.siat.ac.cn/~huihuang/EAR/EAR_TOG.pdf
]]-- 

classifyPoints = CPlane.CPlane.classifyPoints
ArcIO = data.ArcIO

job_id = 'precise-transit-6548'
work_id = 'bilateral-filtering'

arc_io = ArcIO.new( job_id, work_id )
pc = arc_io:getScan( 1 )

points_map = pc:get_xyz_map()

sub_map = points_map:sub(1,3,530,570,35,55):clone()

window = 3
dist_thresh = 9.0

cull_map, eigenvalues, means, normals, second_moments = classifyPoints( sub_map, window, dist_thresh )

image.display(normals)

arc_io:dumpImage( torch.add(normals,1):mul(0.5), 'normals', 'orig_normals' )

height = sub_map:size(2)
width = sub_map:size(3)

sigma_distance = 50
sigma_normal = math.pi/8

function spatial_weight( distance, sigma_distance )
	return math.exp( -math.pow(distance,2)/math.pow(sigma_distance,2) )
end

function normal_weight( n_i, n_ip, sigma_normal )
	return math.exp( -math.pow((1-(n_i:t()*n_ip):squeeze())/(1-math.cos(sigma_normal)),2) )
end

new_normals = torch.Tensor( normals:size() ):zero()

for iter = 1,5 do 

for i = 2,height-1 do 	
	for j = 2,width-1 do 
		print(string.format("[%d,%d]",i,j))					
		p_i = sub_map[{{1,3},{i},{j}}]:squeeze():reshape(3,1)
		n_i = normals[{{1,3},{i},{j}}]:squeeze():reshape(3,1)
		den_i = 0
		num_i = torch.zeros(3,1)
		for k = 2,height-1 do 
			for l = 2,width-1 do 
				p_ip = sub_map[{{1,3},{k},{l}}]:squeeze():reshape(3,1)
				n_ip = normals[{{1,3},{k},{l}}]:squeeze():reshape(3,1)				
				dist = math.sqrt((p_i - p_ip):pow(2):sum())				
				if dist < sigma_distance then
					--print(string.format("[%d,%d]:[%d,%d]",i,j,k,l))					
					neighbor_weight = spatial_weight( dist, sigma_distance )*normal_weight( n_i, n_ip, sigma_normal )
					den_i = den_i + neighbor_weight					
					num_i = num_i + n_ip:mul(neighbor_weight)
				end
			end
		end
		new_normals[{{1,3},{i},{j}}] = num_i/den_i
	end
end
normals = new_normals:clone()
arc_io:dumpImage( new_normals:add(1):mul(0.5), 'normals', string.format('new_normals%.3d', iter) )
collectgarbage()
end














