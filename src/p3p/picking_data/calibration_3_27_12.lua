require 'torch'
local imagedir = "../data/test/96_spring_kitchen/"
local sweep_1 = {
  {
    image_path = imagedir .. "sweep_1/DSC_0180.jpg",
    lens = { lens_type= "thoby", projection= "nil"},
    white_wall = false,
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {0.9205, 2.1464, 0.0227, -0.36940298507463, -0.564039408867},
      {0.9205, 2.1464, 2.3773, -0.41417910447761, 0.26847290640394},
      {3.6412, 0.651, 2.3773, 0.83208955223881, 0.17980295566502}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.37313432835821, 0.20443349753695},
      {3.6412, 0.651, 2.3773, -0.31716417910448, 0.17241379310345},
      {4.0173, -2.4003, 0.1266, 0.61567164179104, -0.28571428571429}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, -0.73880597014925, 0.20689655172414},
      {3.1454, -3.2755, 2.7393, -0.13432835820896, 0.20689655172414},
      {0.3181, -2.3057, 2.313, 0.72014925373134, 0.25369458128079}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {0.506, -2.0305, 0.1266, -0.5, -0.56896551724138},
      {0.3181, -2.3057, 2.313, -0.42910447761194, 0.2487684729064},
      {1.6481, -3.0848, 2.313, -0.91417910447761, 0.17241379310345}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, 0.40671641791045, 0.50492610837438},
      {-1.9099, -0.2201, 0.0226, 0.44776119402985, -0.63300492610837},
      {-2.4256, -0.848, 2.4454, 0.30970149253731, 0.26600985221675}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-1.6858, -0.4453, 2.7393, -0.60074626865672, 0.51231527093596},
      {-1.4034, 0.472, 0.3765, 0.085820895522388, -0.64532019704433},
      {-1.9099, -0.2201, 0.0226, -0.47014925373134, -0.63546798029557}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-0.6529, 1.318, 2.7393, 0.12686567164179, 0.54187192118227},
      {-0.7046, 1.8375, 2.4537, 0.14925373134328, 0.33497536945813},
      {-0.278, 1.4789, 0.3765, 0.40298507462687, -0.64039408866995}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-0.6529, 1.318, 2.7393, -0.58955223880597, 0.55665024630542},
      {-0.1426, 2.7691, 2.3773, -0.16791044776119, 0.22660098522167},
      {0.9205, 2.1464, 0.0227, 0.44029850746269, -0.564039408867}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {3.5983, 0.5777, 0.0227, 0.11567164179104, -0.73399014778325},
      {3.5983, 0.5777, 2.3773, 0.13805970149254, 0.39655172413793},
      {4.6614, -0.0451, 2.3773, 0.63805970149254, 0.24137931034483}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.27985074626866, 0.33251231527094},
      {4.6614, -0.0451, 2.3773, -0.5, 0.23891625615764},
      {3.7318, -2.8875, 0.1266, 0.78731343283582, -0.38669950738916}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {3.1454, -3.2755, 2.7393, 0.029850746268657, 0.25862068965517},
      {1.1721, -2.4207, 0.1266, 0.77238805970149, -0.46305418719212},
      {4.0225, -2.3913, 0.1266, -0.59328358208955, -0.435960591133}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {1.5193, -2.6242, 0.1266, -0.44029850746269, -0.4384236453202},
      {0.3181, -2.3057, 2.313, 0.1455223880597, 0.19211822660099},
      {1.6481, -3.0848, 2.313, -0.53358208955224, 0.18226600985222}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-1.9099, -0.2201, 0.0226, 0.10820895522388, -0.33497536945813},
      {-0.6529, 1.318, 2.7393, 0.79477611940298, 0.3128078817734},
      {-1.6858, -0.4453, 2.7393, 0.07089552238806, 0.24630541871921}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-0.7046, 1.8375, 2.4537, -0.20149253731343, 0.20443349753695},
      {-0.6529, 1.318, 2.7393, -0.3134328358209, 0.30295566502463},
      {0.9205, 2.1464, 0.0227, 0.45522388059702, -0.58374384236453}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {0.9205, 2.1464, 0.0227, -0.46268656716418, -0.58620689655172},
      {0.8891, 2.1648, 2.3409, -0.52611940298507, 0.26600985221675},
      {-0.1402, 2.801, 2.4537, -0.78731343283582, 0.19458128078818}
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
    pairs_calibrated = 1,
    calibration_pairs = torch.Tensor({
      {3.5983, 0.5777, 2.3773, 0.77985074626866, 0.40886699507389},
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {3.2391, -3.6241, 2.4537, 0.70522388059702, 0.44827586206897},
      {3.2731, -3.6515, 0.1266, 0.54477611940298, -0.72167487684729},
      {4.17, -2.7324, 2.3005, -0.52611940298507, 0.41871921182266}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {3.2391, -3.6241, 2.4537, -0.32089552238806, 0.4384236453202},
      {3.0578, -3.5254, 0.512, -0.13805970149254, -0.64285714285714},
      {1.9548, -3.2645, 2.313, 0.86940298507463, 0.36699507389163}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {1.687, -3.1076, 2.2679, 0.078358208955224, 0.30295566502463},
      {1.6481, -3.0848, 1.4047, 0.12686567164179, -0.15024630541872},
      {0.506, -2.0305, 0.1266, 0.82835820895522, -0.50246305418719}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-0.6529, 1.318, 2.7393, 0.76865671641791, 0.18965517241379},
      {-1.6858, -0.4453, 2.7393, 0.23880597014925, 0.1871921182266},
      {1.2593, -2.857, 2.313, -0.70149253731343, 0.29310344827586}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {-0.6529, 1.318, 2.7393, -0.32462686567164, 0.18226600985222},
      {0.9205, 2.1464, 0.0227, 0.15298507462687, -0.30049261083744},
      {-0.7046, 1.8375, 2.4537, -0.28358208955224, 0.12315270935961}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {4.6614, -0.0451, 2.3773, 0.60820895522388, 0.23891625615764},
      {3.5983, 0.5777, 0.0227, -0.048507462686567, -0.47783251231527},
      {3.6297, 0.5593, 2.3409, -0.029850746268657, 0.21674876847291}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {4.1784, -1.5122, 2.7393, 0.059701492537313, 0.59605911330049},
      {4.6614, -0.0451, 0.0227, -0.37313432835821, -0.50492610837438},
      {4.5197, -1.5426, 0.1266, 0.2089552238806, -0.69458128078818}
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
    pairs_calibrated = 3,
    calibration_pairs = torch.Tensor({
      {3.9631, -3.0856, 2.3005, 0.67164179104478, 0.4064039408867},
      {3.7282, -2.8937, 0.7483, 0.60074626865672, -0.69950738916256},
      {5.0208, -1.8362, 0.9317, -0.58208955223881, -0.33497536945813}
    }),
    pose_position = torch.Tensor({3.07656, -2.16894, 1.46938}),
    pose_rotation = torch.Tensor({-0.000783929, -0.000827654, -0.780695, 0.624911}),
    photo_position = torch.Tensor({3.2773886804716, -2.1228451413886, 1.6692377121595}),
    photo_rotation = torch.Tensor({0.00037804136920242, 0, -0.7999390761724, 0.60008110409942})
  }
}

local scan = {sweep_1, sweep_2, sweep_3}
return scan
