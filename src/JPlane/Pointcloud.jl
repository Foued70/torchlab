# Super simple pointcloud module, loads xyz and creates an xyz map
module Pointcloud 

type pointcloud
	data_path::ASCIIString
	xyz_list::Array
	xyz_map::Array
	depth_list::Array
	depth_map::Array
	width::Int
	height::Int

	function pointcloud( data_path::ASCIIString )
		print("Loading pointcloud from: $data_path\n")
		local data_fh = open( "$data_path/sweep.xyz" )
		local meta_fh = open( "$data_path/sweep.meta" )
		# Read in xyz data deliminated by spaces
		local xyz_list = readdlm(data_fh, ' ')
		local depth_list = sum(xyz_list.^2,2)

		# This is totally relies on structur of the metafile ... not good
		local width = int(readline(meta_fh)[3:end-1])
		local height = int(readline(meta_fh)[3:end-1])

		# I think these are all upside-down ... gotta fix that. 
		local xyz_map = reshape( xyz_list, height, width, 3 )		
		local depth_map = reshape( depth_list, height, width )

		# Return new Pointcloud
		new( data_path, xyz_list, xyz_map, depth_list, depth_map, width, height )
	end
end

type Plane 
	equation::Array
	centroid::Array	
	eigenvalues::Array
	curvature::Float64
end

function extract_plane( covariance::Array, centroid::Array )		
	U,S,V = svd( covariance )
	# Flip eigenvalues so that they are min->max	
	local eigenvalues = flipud(S)
	# Estimate of curvature 
	local curvature = eigenvalues[1]/sum(eigenvalues)
	# Normal is corresponds to smallest eigenvalue
	local normal = V[:,3]	
	local d = dot( normal, centroid )
	# Flip normal if required
	if ( d < 0.0 )
		normal = -1.0*normal
		d = -1.0*d
	end
	return Plane( [normal; d], centroid, eigenvalues, curvature )
end

# Hacky little covariance computation, could probably use https://github.com/JuliaStats/StatsBase.jl for this
function compute_covariance( map::Array, centroid::Array, i::Int, j::Int, window_width::Int )
	# Fast
	covariance = zeros(3,3)	
	pos = map[i-window_width:i+window_width,j-window_width:j+window_width,:]
	vdiff = broadcast(-, reshape(pos,9,3), centroid')
	covariance = vdiff'*vdiff	
	"""
	Slow 
	for k = i-window_width:i+window_width
		for l = j-window_width:j+window_width
			pos = map[k,l,:][:]
			vdiff = pos - centroid		
			covariance += vdiff*vdiff'
		end
	end
	"""	
	return covariance
end

# compute normals for pointcloud using local neighborhood 
function compute_normals( pc::pointcloud )
	local width = pc.width
	local height = pc.height
	local xyz_map = pc.xyz_map
	local normal_map = zeros(height, width, 3)
	local window_width = 1
	for i = 1:height
		for j = 1:width
			# Bounds check 
			if i-window_width < 1 || j-window_width < 1 || i+window_width > height || j+window_width > width
				continue
			end			
			# Compute centroid of points ... probably a more julia-y way to tdo this
			seed_centroid = mean(mean(xyz_map[i-window_width:i+window_width,j-window_width:j+window_width,:],1),2)[:]
			# Compute covariance
			seed_covariance = compute_covariance( xyz_map, seed_centroid, i, j, window_width)			
			plane = extract_plane( seed_covariance, seed_centroid )
			normal_map[i,j,:] = plane.equation[1:3]
		end
	end
	return normal_map
end

# Note: this probably isn't the best remapping to show depth or xyz values 
function imagesc( map::Array )
	local mx
	local mn
	local map_scaled
	if length(size(map)) == 2 
		mx = maximum(map) 
		mn = minimum(map) 
		map_scaled = 1.0/(mx-mn)*(map - mn)
	elseif length(size(map)) == 3 
		mx = maximum(maximum(map,1),2) 
		mn = minimum(minimum(map,1),2) 
		map_scaled = broadcast(*, broadcast(-, map, mn ), 1.0/(mx-mn))
	end
	return map_scaled
end

end # Pointcloud  





