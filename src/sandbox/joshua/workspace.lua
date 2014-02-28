_G.pc=pointcloud.loader.load_pobot_ascii("/Users/joshua/Documents/Projects/precise-transit-6548/source/po_scan/a/007")
_G.pk = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/pancakes.lua"
_G.pv = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/planes_verification.lua"
_G.data = torch.load("/Users/joshua/Downloads/toJosh/scan007.t7")

_G.P = pk.viewPlanesFaceOn()