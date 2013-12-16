-- require "./Plane/compare_matchers.lua"
local pc = PointCloud.PointCloud.new("/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia/OD/sweep_001.od")
local location = "/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia/planes3/sweep_001/saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var"
local path = require "path"
local planes = torch.load(path.join(location, "planes.t7"))
table.sort(planes, function(a,b) return a.n_pts>b.n_pts end)
local matcher = Plane.Matcher.new()
local score1, ord1 = matcher:matchDistance(pc,planes)
local ord1_better = Plane.Matcher.removeNoise(score1,ord1,planes, pc:get_xyz_map(),.58) --equivalent to .7 with two variables

local points = pc:get_xyz_map()
local normals, temp, temp, temp, maskN = pc:get_normal_map()
local score2, ord2 = matcher:match(planes, points:reshape(3,points:size(2)*points:size(3)),normals:reshape(3,normals:size(2)*normals:size(3)))
local points = pc:get_xyz_map() score2 = score2:squeeze():reshape(points:size(2), points:size(3)) ord2 = ord2:squeeze():reshape(points:size(2), points:size(3))
local ord2_better = Plane.Matcher.removeNoise(score1,ord1,planes, pc:get_xyz_map(),.7) --equivalent to .7 with two variables

_G.score1 = score1
_G.score2 = score2
_G.planes = planes
_G.ord1 = ord1
_G.ord2 = ord2
_G.ord1_better = ord1_better
_G.ord2_better = ord2_better
html = require "./sandbox/sbraun/CreateHTML.lua"
util.fs.mkdir_p("results/images")
html_doc = html.new("results/compare_matchers.html", "comparison of matching techniques", "using distance as well for matching techniques")
html_doc:beginFile()
for i=1,#planes do
    html_doc:newRow()
  html_doc:addImageFromTensor("original matcher", torch.gt(score2,.7):cmul(torch.eq(ord2,i)):double(), "images/origplane_" .. i .. ".png", "using original matcher with plane "..i)
  html_doc:addImageFromTensor("original matcher", torch.gt(score1,.58):cmul(torch.eq(ord1,i)):double(), "images/displane_" .. i .. ".png", "using distance matcher with plane "..i)
  html_doc:addImageFromTensor("original matcher", torch.eq(ord2_better,i):double(), "images/noise_removed_origplane_" .. i .. ".png", "using original matcher with noise removal on plane "..i)
  html_doc:addImageFromTensor("original matcher", torch.eq(ord1_better,i):double(), "images/noise_removed_displane_" .. i .. ".png", "using distance matcher with noise removal plane "..i, nil, 10)

  html_doc:endRow()

end
    html_doc:endFile()
    html_doc:close_file()
