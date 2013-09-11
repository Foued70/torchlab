p = PointCloud.PointCloud.new("pts.xyz")

t = octomap.Tree.new()

t:add_points(p.points,torch.Tensor({0,0,65.5}),7)

t:stats()
