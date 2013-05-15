require 'torch'
imagedir = "../data/test/"
local sweep_1 = {
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/DSC_0181.jpg",
    lens = { lens_type= "scaramuzza_r2t", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.34350132625995, 0.21320490367776},
      {3.5983, 0.5777, 2.3773, -0.30159151193634, 0.1846234676007},
      {4.2733, -1.9631, 0.1266, 0.45034482758621, -0.25306479859895},
      {4.511, -1.5375, 0.8963, 0.33007957559682, -0.12129597197898}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.18604755023479, 0.08819444133795, 1.6651101229095}),
    photo_rotation = torch.Tensor({-0.00052112516733994, 0, -0.72896808868866, 0.68454748126201})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/DSC_0182.jpg",
    lens = { lens_type= "scaramuzza_r2t", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.1454, -3.2755, 2.7393, -0.14100795755968, 0.21485113835377},
      {4.1784, -1.5122, 2.7393, -0.72381962864721, 0.21828371278459},
      {0.4671, -2.0973, 0.1266, 0.51209549071618, -0.51852889667251},
      {4.0225, -2.3913, 0.1266, -0.59724137931034, -0.2584238178634}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.19417092984233, -0.069194551992786, 1.6648642825758}),
    photo_rotation = torch.Tensor({-0.0022021510119796, 0, -0.93544103511653, 0.35347591198134})
  },
  {
     image_path = imagedir .. "96_spring_kitchen/sweep_1/DSC_0183.jpg",
    lens = { lens_type= "scaramuzza_r2t", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {0.3181, -2.3057, 2.313, -0.42472148541114, 0.25737302977233},
      {0.506, -2.0305, 0.1266, -0.49915119363395, -0.53295971978984},
      {0.506, -2.0305, 0.8887, -0.55305039787798, -0.31005253940455},
      {1.2593, -2.857, 1.4047, -0.80620689655172, -0.060560420315235}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.088624087767656, -0.18622957298258, 1.6647505048929}),
    photo_rotation = torch.Tensor({-0.0236826634438, 0, 0.99922567099071, 0.031419578058621})
  }
}
local scan = {sweep_1}
return scan
