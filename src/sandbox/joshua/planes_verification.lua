local io = require 'io'

t = {}
function t.load_pts(filename)

	--returns something like " 1234 filename"
	cmd_out = util.fs.exec('wc -l '..filename)

	num_rows = tonumber(string.match(cmd_out,"%d+"))
	data = {pts=torch.Tensor(num_rows,3),colors=torch.Tensor(num_rows,3)}
	f_in = io.open(filename)
	for i=1,num_rows do
		data.pts[i][1] = f_in:read("*n")
		data.pts[i][2] = f_in:read("*n")
		data.pts[i][3] = f_in:read("*n")
		data.colors[i][1] = f_in:read("*n")
		data.colors[i][2] = f_in:read("*n")
		data.colors[i][3] = f_in:read("*n")
	end
	return data
end

pc=pointcloud.loader.load_pobot_ascii("/Users/joshua/Documents/Projects/precise-transit-6548/source/po_scan/a/007")

function t.verify_uriahs_plane_points(planes)
	correctness = {}

	for i,p in ipairs(planes) do
		correctness[i] = torch.Tensor(p.mask:size())

		correctness[i] = torch.sum(torch.cmul(pc.xyz_map,p.eqn[{{1,3}}]:reshape(3,1,1):expandAs(pc.xyz_map)),1):reshape(p.mask:size()):add(torch.Tensor(p.mask:size()):fill(p.eqn[4])):cmul(p.mask:double())

	end
	return correctness
end
return t