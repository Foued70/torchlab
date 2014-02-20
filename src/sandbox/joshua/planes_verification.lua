--[==[ USAGE

> pv = require "/Users/joshua/Documents/GitDepot/depot/cloudlab/src/sandbox/joshua/planes_verification.lua"
pointcloud:load_input() - 950.36675399542
> pc = pointcloud.loader.load_pobot_ascii("/Users/joshua/Documents/Projects/precise-transit-6548/source/po_scan/a/007")
> data = torch.load("scan007.t7")
> C = pv.verify_uriahs_plane_points(data.planes,pc)
> C = pv.verify_uriahs_plane_points(data.planes)
> C = pv.verify_uriahs_plane_points() --defaults to 007 scan of precise-blahblah
> C(1)
stdv=	4.487223889121
mean=	1.0098173892399e-11
min=	-95.514856876031
max=	46.733520166387
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 630x880]

> C({1,6})
stdv=	5.7332889976337
mean=	9.2788149714601e-12
min=	-95.514856876031
max=	73.348317403277
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 630x880]

> image.display(C{1,61})
stdv=	7.0361046392574
mean=	7.9095214008346e-12
min=	-141.64818813362
max=	91.148341509167
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 3x630x880]
]==]

local io = require 'io'

t = {}

local pnt_cloud=pointcloud.loader.load_pobot_ascii("/Users/joshua/Documents/Projects/precise-transit-6548/source/po_scan/a/007")
local data=torch.load("/Users/joshua/Downloads/toJosh/scan007.t7")

function t.verify_uriahs_plane_points(planes,pc)
	planes = planes or data.planes
	pc = pc or pnt_cloud

	local correctness = {}

	for i,p in ipairs(planes) do

		correctness[i] = torch.sum(torch.cmul(pc.xyz_map,p.eqn[{{1,3}}]:reshape(3,1,1):expandAs(pc.xyz_map)),1):reshape(p.mask:size()):add(torch.Tensor(p.mask:size()):fill(p.eqn[4])):cmul(p.mask:double())

	end
	return function (ind)

				if type(ind)=="table" and #ind==2 then
					result = torch.zeros(correctness[1]:size())
					for i=ind[1],ind[2] do
						result = result:add(correctness[i])
					end
				elseif type(ind)=="number" then
					result = correctness[ind]
				else
					print("enter index as a number or a table range {start,end}")
					return nil
				end
				print("stdv=",torch.std(result))
				print("mean=",torch.mean(result))
				print("min=",result:min())
				print("max=",result:max())
				return result
			end
end
return t