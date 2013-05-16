require 'torch'
local imagedir = "tests/"
local sweep_1 = {
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0180.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({1.246, 1.989, 2.3771, -0.36756756756757, 0.22142857142857, 3.3912, 0.7323, 2.3771, 0.83783783783784, 0.18214285714286, 0.9205, 2.1464, 0.0227, -0.37297297297297, -0.57142857142857}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.069012514546819, 0.19374106928473, 1.6653440159606}),
    photo_rotation = torch.Tensor({0.00030053433637039, 0, -0.41151428342902, 0.91140326102829})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0181.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({4.1784, -1.5122, 2.7393, 0.32432432432432, 0.175, 4.0173, -2.4003, 0.1266, 0.57297297297297, -0.26785714285714, 3.6297, 0.5593, 0.0591, -0.24864864864865, -0.34285714285714}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.18604755023479, 0.08819444133795, 1.6651101229095}),
    photo_rotation = torch.Tensor({-0.00052112516733994, 0, -0.72896808868866, 0.68454748126201})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0182.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({4.1784, -1.5122, 2.7393, -0.7027027027027, 0.19642857142857, 0.506, -2.0305, 0.1266, 0.53513513513514, -0.59642857142857, 4.0225, -2.3913, 0.1266, -0.42702702702703, -0.25714285714286}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.19417092984233, -0.069194551992786, 1.6648642825758}),
    photo_rotation = torch.Tensor({-0.0022021510119796, 0, -0.93544103511653, 0.35347591198134})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0183.jpg",
    white_wall = true,
    pairs_calibrated = 1,
    calibration_pairs = torch.Tensor({0.506, -2.0305, 0.1266, -0.52972972972973, -0.6, 0.2307, -2.482, 2.4537, 0, 0, 0, 0, 0, 0, 0}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.088624087767656, -0.18622957298258, 1.6647505048929}),
    photo_rotation = torch.Tensor({-0.0236826634438, 0, 0.99922567099071, 0.031419578058621})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0184.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-1.6858, -0.4453, 2.7393, 0.4, 0.49285714285714, -1.9099, -0.2201, 0.0226, 0.49189189189189, -0.69285714285714, -2.4297, -0.8551, 2.4537, 0.27027027027027, 0.22857142857143}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.06876506736755, -0.19435309360824, 1.6648354392842}),
    photo_rotation = torch.Tensor({-0.00066560970591633, 0, 0.91140301797656, 0.41151439317106})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0185.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-1.9099, -0.2201, 0.0226, -0.51891891891892, -0.70357142857143, -1.6858, -0.4453, 2.7393, -0.62702702702703, 0.53571428571429, -2.4297, -0.8551, 2.4537, -0.77297297297297, 0.26785714285714}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.18580010305552, -0.088806465661461, 1.6650693323353}),
    photo_rotation = torch.Tensor({0.00048936961041886, -0, 0.68454730634161, 0.72896827495977})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0186.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-0.6529, 1.318, 2.7393, 0.12432432432432, 0.52142857142857, -0.7046, 1.8375, 2.4537, 0.12972972972973, 0.29285714285714, -0.3065, 1.4956, 0.3765, 0.42702702702703, -0.68928571428571}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.19392348266307, 0.068582527669275, 1.665315172669}),
    photo_rotation = torch.Tensor({0.00083212639934159, -0, 0.35347493251527, 0.93544362718925})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_1/JPG/DSC_0187.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-0.6529, 1.318, 2.7393, -0.63243243243243, 0.59642857142857, -0.7046, 1.8375, 2.4537, -0.60540540540541, 0.32857142857143, 0.9205, 2.1464, 0.0227, 0.45405405405405, -0.58214285714286}):resize(3,5),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.088376640588399, 0.18561754865907, 1.6654289503519}),
    photo_rotation = torch.Tensor({0.0007444668485139, 0, -0.031410756986921, 0.99950628317916})
  }
}

local sweep_2 = {
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0188.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({3.5983, 0.5777, 0.0227, 0.12972972972973, -0.84285714285714, 3.6297, 0.5593, 2.3409, 0.14054054054054, 0.32857142857143, 4.5855, -0.046, 0.0461, 0.60540540540541, -0.53928571428571}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.3018725386043, 0.27422364975589, 1.6635647669392}),
    photo_rotation = torch.Tensor({0.00059874144799109, 0, -0.65573455941021, 0.75499127750179})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0189.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({4.1784, -1.5122, 2.7393, 0.24864864864865, 0.29285714285714, 3.7318, -2.8875, 0.1266, 0.82162162162162, -0.40714285714286, 4.6135, -1.2779, 2.4537, 0.1027027027027, 0.19285714285714}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.34160858819, 0.12171665073472, 1.6634837220905}),
    photo_rotation = torch.Tensor({0.00022371220233756, 0, -0.89474256132848, 0.4465822420341})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0190.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({3.1454, -3.2755, 2.7393, 0.027027027027027, 0.22142857142857, 2.4061, -3.1101, 0.1276, 0.27027027027027, -0.35, 4.0225, -2.3913, 0.1266, -0.38378378378378, -0.37857142857143}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.2618675075025, -0.014219649347923, 1.6633184162819}),
    photo_rotation = torch.Tensor({-0.0044276174652419, 0, -0.99752410311124, 0.070185895418513})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0191.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({0.506, -2.0305, 0.1266, 0.11351351351351, -0.41071428571429, 1.6481, -3.0848, 1.4047, -0.47027027027027, -0.064285714285714, 1.1783, -2.4244, 0.1266, -0.25945945945946, -0.41428571428571}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.1093605401303, -0.053955609522461, 1.6631656834141}),
    photo_rotation = torch.Tensor({-0.0017020698454097, 0, 0.94845826597862, 0.31689749234583})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0192.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-1.6858, -0.4453, 2.7393, 0.05945945945946, 0.20714285714286, -0.6529, 1.318, 2.7393, 0.8, 0.31785714285714, -1.9099, -0.2201, 0.0226, 0.091891891891892, -0.29285714285714}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({1.9734241992037, 0.025785556767438, 1.6631149923297}),
    photo_rotation = torch.Tensor({-0.00068937098980796, 0, 0.75499096277468, 0.65573483276108})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0193.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-0.7046, 1.8375, 2.4537, -0.16756756756757, 0.17142857142857, -0.1063, 2.7025, 0.0461, 0.21621621621622, -0.36071428571429, 0.8991, 2.1922, 2.4537, 0.5027027027027, 0.26428571428571}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({1.933688149618, 0.1782925557886, 1.6631960371784}),
    photo_rotation = torch.Tensor({-0.0001116588145971, 0, 0.44658222807511, 0.89474258929581})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0194.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({0.8891, 2.1648, 2.3409, -0.47567567567568, 0.24642857142857, 0.9205, 2.1464, 0.0227, -0.48648648648649, -0.62142857142857, -0.1402, 2.801, 2.4537, -0.76216216216216, 0.18928571428571}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.0134292303055, 0.31422885587125, 1.663361342987}),
    photo_rotation = torch.Tensor({0.00031152453790723, -0, 0.070185204054468, 0.99753392928977})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_2/JPG/DSC_0195.jpg",
    white_wall = true,
    pairs_calibrated = 0,
    calibration_pairs = torch.Tensor({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}):resize(3,5),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.1659361976777, 0.35396481604579, 1.6635140758548}),
    photo_rotation = torch.Tensor({0.0005686921782732, 0, -0.31689698206875, 0.94845979321473})
  }
}

local sweep_3 = {
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0196.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({3.2391, -3.6241, 2.4537, 0.73513513513514, 0.46785714285714, 3.284, -3.6665, 0.9317, 0.67567567567568, -0.45, 3.2731, -3.6515, 0.1266, 0.68648648648649, -0.91428571428571}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.2508512410141, -2.2781930770159, 1.6695957048493}),
    photo_rotation = torch.Tensor({0.003827824910638, 0, -0.96868046528187, 0.24828109863168})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0197.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({3.2391, -3.6241, 2.4537, -0.2972972972973, 0.40714285714286, 3.2731, -3.6515, 0.1266, -0.34054054054054, -0.825, 3.0653, -3.5298, 0.512, -0.14594594594595, -0.67142857142857}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.1222385692513, -2.3692758531225, 1.669827036059}),
    photo_rotation = torch.Tensor({0.0079049995219887, -0, 0.98993158569939, 0.14132574647688})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0198.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({0.506, -2.0305, 0.1266, 0.95675675675676, -0.57857142857143, 1.8922, -2.8171, 0.1276, 0.12432432432432, -0.88928571428571, 1.6481, -3.0848, 1.4047, 0.10810810810811, -0.125}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.9668902240089, -2.3427384147638, 1.6697961951032}),
    photo_rotation = torch.Tensor({0.0012358568980952, -0, 0.86052467571308, 0.50940725863166})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0199.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-0.6529, 1.318, 2.7393, 0.74054054054054, 0.18214285714286, 0.506, -2.0305, 0.1266, -0.14594594594595, -0.44642857142857, 1.2593, -2.857, 1.4047, -0.67027027027027, -0.12857142857143}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.8758071590375, -2.2141260334197, 1.6695212481956}),
    photo_rotation = torch.Tensor({-0.00028359091792199, 0, 0.60008103708861, 0.79993916550126})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0200.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({-0.6529, 1.318, 2.7393, -0.27567567567568, 0.15357142857143, -0.7046, 1.8375, 2.4537, -0.23783783783784, 0.10357142857143, 0.9205, 2.1464, 0.0227, 0.12972972972973, -0.25714285714286}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.902344598495, -2.0587780977924, 1.6691632555057}),
    photo_rotation = torch.Tensor({-0.00098109664460922, 0, 0.24827916019498, 0.96868802824349})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0201.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({3.5983, 0.5777, 0.0227, -0.043243243243243, -0.44285714285714, 4.6687, -0.0161, 2.3771, 0.55675675675676, 0.21785714285714, 4.6614, -0.0451, 0.0227, 0.55675675675676, -0.52142857142857}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.0309572702578, -1.9676953216858, 1.6689319242961}),
    photo_rotation = torch.Tensor({-0.0011285066127769, 0, -0.14132124076047, 0.98996314748719})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0202.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({4.1784, -1.5122, 2.7393, 0.05945945945946, 0.59285714285714, 4.6135, -1.2779, 2.4537, 0.1027027027027, 0.325, 4.5197, -1.5426, 0.1266, 0.23243243243243, -0.76785714285714}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.1863056155003, -1.9942327600445, 1.6689627652519}),
    photo_rotation = torch.Tensor({-0.00073159299557269, 0, -0.5094067332872, 0.86052556316088})
  },
  {
    image_path = imagedir .. "96_spring_kitchen/sweep_3/JPG/DSC_0203.jpg",
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({4.17, -2.7324, 1.3922, 0.24864864864865, -0.18571428571429, 4.5886, -2.0178, 1.3922, -0.43783783783784, -0.16428571428571, 3.9373, -3.1297, 2.2554, 0.74054054054054, 0.375}):resize(3,5),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.2773886804716, -2.1228451413886, 1.6692377121595}),
    photo_rotation = torch.Tensor({0.00037804136920242, 0, -0.7999390761724, 0.60008110409942})
  }
}

local scan = {sweep_1, sweep_2, sweep_3}
return scan
