-- test data from Nick B. in world: x,y,z camera: u,v coordinates
cameras = 
{{
    model = "objects/rivercourt_3307_v3.obj",
    photo = "texture/_DSC5436.jpg",
    sweep = sweep[1],
    lens  = nikon_D800E_w18mm,
    world = torch.Tensor({{ 2.54144, -1.13234,  -0.01428},
                          { 2.92356,  0.198475, -0.006072},
                          { 1.90087,  0.593343,  2.54924}}),
    uv    = torch.Tensor({{ 0.6575,   0.723333,  1},
                          { 0.3225,   0.783333,  1},
                          { 0.155,    0.213333,  1}})
 },
 {
    model = "objects/rivercourt_3307_v3.obj",
    photo = "texture/_DSC5445.jpg",
    sweep = sweep[1],
    lens  = nikon_D800E_w18mm,
    world = torch.Tensor({{ 2.97666, -2.91019, -0.023381}, 
                          { 2.54144, -1.13234, -0.01428},
                          { 3.44288, -1.38701,  2.54188}}),
    uv    = torch.Tensor({{ 0.5,      0.686667, 1},
                          { 0.1925,   0.73,     1},
                          { 0.1725,   0.355,    1}})
 },
 {
    model = "objects/rivercourt_3307_v3.obj",
    photo = "texture/_DSC5526.jpg",
    sweep = sweep[1],
    lens  = nikon_D800E_w18mm,
    world = torch.Tensor({{ 1.51994, 1.22472, -0.018862},
                          { 1.98344, 0.882942, 2.54038}, 
                          { 1.90087, 0.593343, 2.54924}}),
    uv    = torch.Tensor({{ 0.41,    0.801667, 1},
                          { 0.3425,  0.256667, 1},
                          { 0.4675,  0.24,     1}})
 },
 {
    model = "objects/rivercourt_3307_v3.obj",
    photo = "texture/_DSC6044.jpg",
    sweep = sweep[7],
    lens  = nikon_D800E_w18mm,
    world = torch.Tensor({{ 2.54144, -1.13234,  -0.01428},
                          { 5.46235, -1.96442,  -0.036059},
                          { 5.8448,  -0.632468, -0.027886}}),
    uv    = torch.Tensor({{ 0.615,    0.341667, 1},
                          { 0.4625,   0.315,    1},
                          { 0.2575,   0.278333, 1}})
 },
 {
    model = "objects/rivercourt_3307_v3.obj",
    photo = "texture/_DSC6044.jpg",
    sweep = sweep[7],
    lens  = nikon_D800E_w18mm,
    world = torch.Tensor({{ 1.49472, 1.16601,  1.99624}, 
                          { 2.53271, 0.85861,  1.99202},
                          { 2.6549,  1.33312,  2.13186}}),
    uv    = torch.Tensor({{ 0.6825,  0.383333, 1},
                          { 0.3675,  0.381667, 1},
                          { 0.3475,  0.355,    1}})
 }
}
