require 'torch'
local imagedir = "/Users/marco/projects/cloudlab/src/data/test/96_spring_kitchen/nikon_10.5mm/"
local sweep_1 = {
  {
    image_path = imagedir .. "sweep_1/DSC_0180.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = true,
    pairs_calibrated = 2,
    calibration_pairs = torch.Tensor({
      {0.9205, 2.1464, 0.0227, -0.36668435013262, -0.56392294220665},
      {0.9205, 2.1464, 2.3773, -0.41570291777188, 0.26893169877408},
      {1.5796, 0.8387, 0.0342, 0, 0},
      {0, 0, 0, 0, 0}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.069012514546819, 0.19374106928473, 1.6653440159606}),
    photo_rotation = torch.Tensor({0.00030053433637039, 0, -0.41151428342902, 0.91140326102829})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0181.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.37374005305039, 0.20448336252189},
      {3.6412, 0.651, 2.3773, -0.31511936339523, 0.17523642732049},
      {4.0173, -2.4003, 0.1266, 0.61628647214854, -0.2860595446585},
      {3.6297, 0.5593, 0.0591, -0.27793103448276, -0.38336252189142}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.18604755023479, 0.08819444133795, 1.6651101229095}),
    photo_rotation = torch.Tensor({-0.00052112516733994, 0, -0.72896808868866, 0.68454748126201})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0182.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.1454, -3.2755, 2.7393, -0.13474801061008, 0.20644483362522},
      {4.1784, -1.5122, 2.7393, -0.7404774535809, 0.2077057793345},
      {0.506, -2.0305, 0.1266, 0.50636604774536, -0.56581436077058},
      {4.0225, -2.3913, 0.1266, -0.47612732095491, -0.28451838879159}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.19417092984233, -0.069194551992786, 1.6648642825758}),
    photo_rotation = torch.Tensor({-0.0022021510119796, 0, -0.93544103511653, 0.35347591198134})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0183.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {0.3181, -2.3057, 2.313, -0.43140583554377, 0.25054290718038},
      {0.506, -2.0305, 0.1266, -0.50201591511936, -0.56910683012259},
      {1.6481, -3.0848, 2.313, -0.91379310344828, 0.17264448336252},
      {0.3181, -2.3057, 1.4047, -0.44010610079576, -0.10697022767075}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({0.088624087767656, -0.18622957298258, 1.6647505048929}),
    photo_rotation = torch.Tensor({-0.0236826634438, 0, 0.99922567099071, 0.031419578058621})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0184.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, 0.40854111405835, 0.50287215411559},
      {-2.4256, -0.848, 2.4454, 0.31177718832891, 0.264588441331},
      {-1.9099, -0.2201, 0.0226, 0.44885941644562, -0.63145359019265},
      {-2.4256, -0.848, 0.0336, 0.27559681697613, -0.49880910683012}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.06876506736755, -0.19435309360824, 1.6648354392842}),
    photo_rotation = torch.Tensor({-0.00066560970591633, 0, 0.91140301797656, 0.41151439317106})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0185.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, -0.60180371352785, 0.5122942206655},
      {-1.9099, -0.2201, 0.0226, -0.46832891246684, -0.63663747810858},
      {-1.4034, 0.472, 0.3765, 0.08371352785146, -0.64406304728547},
      {-2.4256, -0.848, 0.0336, -0.71183023872679, -0.5184938704028}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.18580010305552, -0.088806465661461, 1.6650693323353}),
    photo_rotation = torch.Tensor({0.00048936961041886, -0, 0.68454730634161, 0.72896827495977})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0186.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-0.7046, 1.8375, 2.4537, 0.15103448275862, 0.33481611208406},
      {-0.278, 1.4789, 0.3765, 0.40318302387268, -0.63975481611208},
      {-0.6529, 1.318, 2.7393, 0.12663129973475, 0.5430472854641},
      {-0.2059, 1.5093, 0.1301, 0.42196286472148, -0.70868651488616}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.19392348266307, 0.068582527669275, 1.665315172669}),
    photo_rotation = torch.Tensor({0.00083212639934159, -0, 0.35347493251527, 0.93544362718925})
  },
  {
    image_path = imagedir .. "sweep_1/DSC_0187.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-0.7046, 1.8375, 2.4537, -0.63214854111406, 0.34322241681261},
      {-0.6529, 1.318, 2.7393, -0.58790450928382, 0.55716287215412},
      {-0.1426, 2.7691, 2.3773, -0.16753315649867, 0.22725043782837},
      {0.9205, 2.1464, 0.0227, 0.44122015915119, -0.5630823117338}
    }),
    pose_position = torch.Tensor({0, 0, 1.46509}),
    pose_rotation = torch.Tensor({0.000765031, 0.000309309, 2.4143e-07, 1}),
    photo_position = torch.Tensor({-0.088376640588399, 0.18561754865907, 1.6654289503519}),
    photo_rotation = torch.Tensor({0.0007444668485139, 0, -0.031410756986921, 0.99950628317916})
  }
}

local sweep_2 = {
  {
    image_path = imagedir .. "sweep_2/DSC_0188.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.5983, 0.5777, 2.3773, 0.1385676392573, 0.39677758318739},
      {4.6614, -0.0451, 2.3773, 0.63559681697613, 0.24203152364273},
      {4.5916, -0.0567, 0.0461, 0.58419098143237, -0.51747810858143},
      {3.5983, 0.5777, 0.0227, 0.1151724137931, -0.73352014010508}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.3018725386043, 0.27422364975589, 1.6635647669392}),
    photo_rotation = torch.Tensor({0.00059874144799109, 0, -0.65573455941021, 0.75499127750179})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0189.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.27920424403183, 0.33274956217163},
      {4.6135, -1.2779, 2.4537, 0.11920424403183, 0.22872154115587},
      {4.0225, -2.3913, 0.1266, 0.58970822281167, -0.41145359019264},
      {4.3755, -2.3815, 1.3922, 0.5274801061008, -0.077513134851134}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.34160858819, 0.12171665073472, 1.6634837220905}),
    photo_rotation = torch.Tensor({0.00022371220233756, 0, -0.89474256132848, 0.4465822420341})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0190.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.1454, -3.2755, 2.7393, 0.029496021220158, 0.26056042031524},
      {3.7318, -2.8875, 0.1266, -0.21607427055703, -0.37302977232925},
      {4.3755, -2.3815, 1.3922, -0.55994694960212, -0.079194395796844},
      {1.2593, -2.857, 2.313, 0.80132625994695, 0.19288966725044}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.2618675075025, -0.014219649347923, 1.6633184162819}),
    photo_rotation = torch.Tensor({-0.0044276174652419, 0, -0.99752410311124, 0.070185895418513})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0191.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {1.8664, -2.8275, 0.1266, -0.58923076923077, -0.42718038528897},
      {1.1783, -2.4244, 0.1266, -0.28286472148541, -0.44854640980735},
      {0.9505, -2.6761, 1.4047, -0.19076923076923, -0.074290718038524},
      {1.6481, -3.0848, 2.313, -0.53586206896552, 0.18133099824869}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.1093605401303, -0.053955609522461, 1.6631656834141}),
    photo_rotation = torch.Tensor({-0.0017020698454097, 0, 0.94845826597862, 0.31689749234583})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0192.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, 0.071140583554375, 0.24563922942207},
      {-0.6529, 1.318, 2.7393, 0.79029177718833, 0.31348511383538},
      {-1.9099, -0.2201, 0.0226, 0.1089124668435, -0.33481611208406},
      {-1.0046, 0.2384, 0.3765, 0.2973474801061, -0.34665499124343}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({1.9734241992037, 0.025785556767438, 1.6631149923297}),
    photo_rotation = torch.Tensor({-0.00068937098980796, 0, 0.75499096277468, 0.65573483276108})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0193.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-0.7046, 1.8375, 2.4537, -0.20063660477454, 0.20364273204904},
      {-0.6767, 1.7124, 0.3765, -0.1674801061008, -0.34402802101577},
      {-0.6529, 1.318, 2.7393, -0.31262599469496, 0.3046234676007},
      {0.9205, 2.1464, 0.0227, 0.45851458885941, -0.5830122591944}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({1.933688149618, 0.1782925557886, 1.6631960371784}),
    photo_rotation = torch.Tensor({-0.0001116588145971, 0, 0.44658222807511, 0.89474258929581})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0194.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {0.9205, 2.1464, 0.0227, -0.46180371352785, -0.58693520140105},
      {-0.1125, 2.7009, 0.0461, -0.75331564986737, -0.41943957968477},
      {-0.1402, 2.801, 2.4537, -0.78801061007958, 0.19495621716287},
      {0.9205, 2.1464, 2.3773, -0.51098143236074, 0.28935201401051}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.0134292303055, 0.31422885587125, 1.663361342987}),
    photo_rotation = torch.Tensor({0.00031152453790723, -0, 0.070185204054468, 0.99753392928977})
  },
  {
    image_path = imagedir .. "sweep_2/DSC_0195.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = true,
    pairs_calibrated = 0,
    calibration_pairs = torch.Tensor({
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}
    }),
    pose_position = torch.Tensor({2.13781, 0.150153, 1.46334}),
    pose_rotation = torch.Tensor({0.00047134, -0.000280629, -0.286949, 0.957946}),
    photo_position = torch.Tensor({2.1659361976777, 0.35396481604579, 1.6635140758548}),
    photo_rotation = torch.Tensor({0.0005686921782732, 0, -0.31689698206875, 0.94845979321473})
  }
}

local sweep_3 = {
  {
    image_path = imagedir .. "sweep_3/DSC_0196.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.1454, -3.2755, 2.7393, 0.70604774535809, 0.44795096322242},
      {3.2731, -3.6515, 0.1266, 0.54403183023872, -0.7215411558669},
      {3.5445, -3.8002, 1.3922, 0.54259946949602, -0.15565674255692},
      {4.17, -2.7324, 1.3922, -0.55188328912467, -0.22269702276708}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.2508512410141, -2.2781930770159, 1.6695957048493}),
    photo_rotation = torch.Tensor({0.003827824910638, 0, -0.96868046528187, 0.24828109863168})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0197.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.2391, -3.6241, 2.4537, -0.32318302387268, 0.43891418563923},
      {2.5893, -3.6362, 2.313, 0.24424403183024, 0.35803852889667},
      {2.5893, -3.6362, 1.4047, 0.25931034482759, -0.16865148861646},
      {3.5445, -3.8002, 1.3922, -0.58535809018568, -0.15961471103327}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.1222385692513, -2.3692758531225, 1.669827036059}),
    photo_rotation = torch.Tensor({0.0079049995219887, -0, 0.98993158569939, 0.14132574647688})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0198.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {0.506, -2.0305, 0.1266, 0.82774535809019, -0.50098073555166},
      {2.4412, -3.1533, 0.8963, -0.41761273209549, -0.55628721541156},
      {2.4365, -3.1614, 0.512, -0.36572944297082, -0.71106830122592},
      {2.5893, -3.6362, 2.313, -0.80328912466843, 0.37145359019265}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.9668902240089, -2.3427384147638, 1.6697961951032}),
    photo_rotation = torch.Tensor({0.0012358568980952, -0, 0.86052467571308, 0.50940725863166})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0199.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, 0.23984084880637, 0.18651488616462},
      {-0.6529, 1.318, 2.7393, 0.76896551724138, 0.18935201401051},
      {0.506, -2.0305, 0.1266, -0.15724137931035, -0.47887915936953},
      {1.2394, -2.8909, 1.4047, -0.71989389920424, -0.1369877408056}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.8758071590375, -2.2141260334197, 1.6695212481956}),
    photo_rotation = torch.Tensor({-0.00028359091792199, 0, 0.60008103708861, 0.79993916550126})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0200.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {-0.6529, 1.318, 2.7393, -0.32456233421751, 0.18262697022767},
      {-0.7046, 1.8375, 2.4537, -0.2836074270557, 0.12441330998249},
      {-0.1063, 2.7025, 0.0461, -0.014270557029179, -0.24294220665499},
      {0.8991, 2.1922, 2.4537, 0.16641909814324, 0.13880910683012}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({2.902344598495, -2.0587780977924, 1.6691632555057}),
    photo_rotation = torch.Tensor({-0.00098109664460922, 0, 0.24827916019498, 0.96868802824349})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0201.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {4.6614, -0.0451, 2.3773, 0.60880636604775, 0.23887915936953},
      {3.5983, 0.5777, 2.3773, -0.044615384615384, 0.22714535901926},
      {3.5983, 0.5777, 0.0227, -0.049336870026525, -0.47824868651489},
      {4.6614, -0.0451, 0.0227, 0.54297082228117, -0.50896672504378}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.0309572702578, -1.9676953216858, 1.6689319242961}),
    photo_rotation = torch.Tensor({-0.0011285066127769, 0, -0.14132124076047, 0.98996314748719})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0202.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.060424403183025, 0.5938353765324},
      {4.6135, -1.2779, 2.4537, 0.12010610079576, 0.36802101576182},
      {4.7955, -1.6646, 1.3922, 0.45183023872679, -0.15576182136602},
      {4.511, -1.5375, 0.8963, 0.23628647214854, -0.42753064798599}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.1863056155003, -1.9942327600445, 1.6689627652519}),
    photo_rotation = torch.Tensor({-0.00073159299557269, 0, -0.5094067332872, 0.86052556316088})
  },
  {
    image_path = imagedir .. "sweep_3/DSC_0203.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 4,
    calibration_pairs = torch.Tensor({
      {3.7282, -2.8937, 0.7483, 0.59968169761273, -0.70091068301226},
      {4.0086, -2.3952, 0.8963, -0.073474801061007, -0.6353415061296},
      {5.0208, -1.8362, 0.9317, -0.58397877984085, -0.33509632224168},
      {4.7955, -1.6646, 2.3005, -0.72875331564987, 0.3016112084063}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.2773886804716, -2.1228451413886, 1.6692377121595}),
    photo_rotation = torch.Tensor({0.00037804136920242, 0, -0.7999390761724, 0.60008110409942})
  }
}

local scan = {sweep_1, sweep_2, sweep_3}
return scan
