-- expanded from opencv2/imgproc/types_c.h
cvtColor = {}

cvtColor.BGR2BGRA    = 0
cvtColor.RGB2RGBA    = cvtColor.BGR2BGRA

cvtColor.BGRA2BGR    = 1
cvtColor.RGBA2RGB    = cvtColor.BGRA2BGR

cvtColor.BGR2RGBA    = 2
cvtColor.RGB2BGRA    = cvtColor.BGR2RGBA

cvtColor.RGBA2BGR    = 3
cvtColor.BGRA2RGB    = cvtColor.RGBA2BGR

cvtColor.BGR2RGB     = 4
cvtColor.RGB2BGR     = cvtColor.BGR2RGB

cvtColor.BGRA2RGBA   = 5
cvtColor.RGBA2BGRA   = cvtColor.BGRA2RGBA

cvtColor.BGR2GRAY    = 6
cvtColor.RGB2GRAY    = 7
cvtColor.GRAY2BGR    = 8
cvtColor.GRAY2RGB    = cvtColor.GRAY2BGR
cvtColor.GRAY2BGRA   = 9
cvtColor.GRAY2RGBA   = cvtColor.GRAY2BGRA
cvtColor.BGRA2GRAY   = 10
cvtColor.RGBA2GRAY   = 11

cvtColor.BGR2BGR565  = 12
cvtColor.RGB2BGR565  = 13
cvtColor.BGR5652BGR  = 14
cvtColor.BGR5652RGB  = 15
cvtColor.BGRA2BGR565 = 16
cvtColor.RGBA2BGR565 = 17
cvtColor.BGR5652BGRA = 18
cvtColor.BGR5652RGBA = 19

cvtColor.GRAY2BGR565 = 20
cvtColor.BGR5652GRAY = 21

cvtColor.BGR2BGR555  = 22
cvtColor.RGB2BGR555  = 23
cvtColor.BGR5552BGR  = 24
cvtColor.BGR5552RGB  = 25
cvtColor.BGRA2BGR555 = 26
cvtColor.RGBA2BGR555 = 27
cvtColor.BGR5552BGRA = 28
cvtColor.BGR5552RGBA = 29

cvtColor.GRAY2BGR555 = 30
cvtColor.BGR5552GRAY = 31

cvtColor.BGR2XYZ     = 32
cvtColor.RGB2XYZ     = 33
cvtColor.XYZ2BGR     = 34
cvtColor.XYZ2RGB     = 35

cvtColor.BGR2YCrCb   = 36
cvtColor.RGB2YCrCb   = 37
cvtColor.YCrCb2BGR   = 38
cvtColor.YCrCb2RGB   = 39

cvtColor.BGR2HSV     = 40
cvtColor.RGB2HSV     = 41

cvtColor.BGR2Lab     = 44
cvtColor.RGB2Lab     = 45

cvtColor.BayerBG2BGR = 46
cvtColor.BayerGB2BGR = 47
cvtColor.BayerRG2BGR = 48
cvtColor.BayerGR2BGR = 49

cvtColor.BayerBG2RGB = cvtColor.BayerRG2BGR
cvtColor.BayerGB2RGB = cvtColor.BayerGR2BGR
cvtColor.BayerRG2RGB = cvtColor.BayerBG2BGR
cvtColor.BayerGR2RGB = cvtColor.BayerGB2BGR

cvtColor.BGR2Luv     = 50
cvtColor.RGB2Luv     = 51
cvtColor.BGR2HLS     = 52
cvtColor.RGB2HLS     = 53

cvtColor.HSV2BGR     = 54
cvtColor.HSV2RGB     = 55

cvtColor.Lab2BGR     = 56
cvtColor.Lab2RGB     = 57
cvtColor.Luv2BGR     = 58
cvtColor.Luv2RGB     = 59
cvtColor.HLS2BGR     = 60
cvtColor.HLS2RGB     = 61

cvtColor.BayerBG2BGR_VNG = 62
cvtColor.BayerGB2BGR_VNG = 63
cvtColor.BayerRG2BGR_VNG = 64
cvtColor.BayerGR2BGR_VNG = 65

cvtColor.BayerBG2RGB_VNG = cvtColor.BayerRG2BGR_VNG
cvtColor.BayerGB2RGB_VNG = cvtColor.BayerGR2BGR_VNG
cvtColor.BayerRG2RGB_VNG = cvtColor.BayerBG2BGR_VNG
cvtColor.BayerGR2RGB_VNG = cvtColor.BayerGB2BGR_VNG

cvtColor.BGR2HSV_FULL = 66
cvtColor.RGB2HSV_FULL = 67
cvtColor.BGR2HLS_FULL = 68
cvtColor.RGB2HLS_FULL = 69

cvtColor.HSV2BGR_FULL = 70
cvtColor.HSV2RGB_FULL = 71
cvtColor.HLS2BGR_FULL = 72
cvtColor.HLS2RGB_FULL = 73

cvtColor.LBGR2Lab     = 74
cvtColor.LRGB2Lab     = 75
cvtColor.LBGR2Luv     = 76
cvtColor.LRGB2Luv     = 77

cvtColor.Lab2LBGR     = 78
cvtColor.Lab2LRGB     = 79
cvtColor.Luv2LBGR     = 80
cvtColor.Luv2LRGB     = 81

cvtColor.BGR2YUV      = 82
cvtColor.RGB2YUV      = 83
cvtColor.YUV2BGR      = 84
cvtColor.YUV2RGB      = 85

cvtColor.BayerBG2GRAY = 86
cvtColor.BayerGB2GRAY = 87
cvtColor.BayerRG2GRAY = 88
cvtColor.BayerGR2GRAY = 89

-- YUV 4:2:0 formats family
cvtColor.YUV2RGB_NV12 = 90
cvtColor.YUV2BGR_NV12 = 91
cvtColor.YUV2RGB_NV21 = 92
cvtColor.YUV2BGR_NV21 = 93
cvtColor.YUV420sp2RGB = cvtColor.YUV2RGB_NV21
cvtColor.YUV420sp2BGR = cvtColor.YUV2BGR_NV21

cvtColor.YUV2RGBA_NV12 = 94
cvtColor.YUV2BGRA_NV12 = 95
cvtColor.YUV2RGBA_NV21 = 96
cvtColor.YUV2BGRA_NV21 = 97
cvtColor.YUV420sp2RGBA = cvtColor.YUV2RGBA_NV21
cvtColor.YUV420sp2BGRA = cvtColor.YUV2BGRA_NV21

cvtColor.YUV2RGB_YV12 = 98
cvtColor.YUV2BGR_YV12 = 99
cvtColor.YUV2RGB_IYUV = 100
cvtColor.YUV2BGR_IYUV = 101
cvtColor.YUV2RGB_I420 = cvtColor.YUV2RGB_IYUV
cvtColor.YUV2BGR_I420 = cvtColor.YUV2BGR_IYUV
cvtColor.YUV420p2RGB = cvtColor.YUV2RGB_YV12
cvtColor.YUV420p2BGR = cvtColor.YUV2BGR_YV12

cvtColor.YUV2RGBA_YV12 = 102
cvtColor.YUV2BGRA_YV12 = 103
cvtColor.YUV2RGBA_IYUV = 104
cvtColor.YUV2BGRA_IYUV = 105
cvtColor.YUV2RGBA_I420 = cvtColor.YUV2RGBA_IYUV
cvtColor.YUV2BGRA_I420 = cvtColor.YUV2BGRA_IYUV
cvtColor.YUV420p2RGBA = cvtColor.YUV2RGBA_YV12
cvtColor.YUV420p2BGRA = cvtColor.YUV2BGRA_YV12

cvtColor.YUV2GRAY_420 = 106
cvtColor.YUV2GRAY_NV21 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_NV12 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_YV12 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_IYUV = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_I420 = cvtColor.YUV2GRAY_420
cvtColor.YUV420sp2GRAY = cvtColor.YUV2GRAY_420
cvtColor.YUV420p2GRAY = cvtColor.YUV2GRAY_420

    -- YUV 4:2:2 formats family
cvtColor.YUV2RGB_UYVY = 107
cvtColor.YUV2BGR_UYVY = 108
    -- cvtColor.YUV2RGB_VYUY = 109
    -- cvtColor.YUV2BGR_VYUY = 110
cvtColor.YUV2RGB_Y422 = cvtColor.YUV2RGB_UYVY
cvtColor.YUV2BGR_Y422 = cvtColor.YUV2BGR_UYVY
cvtColor.YUV2RGB_UYNV = cvtColor.YUV2RGB_UYVY
cvtColor.YUV2BGR_UYNV = cvtColor.YUV2BGR_UYVY

cvtColor.YUV2RGBA_UYVY = 111
cvtColor.YUV2BGRA_UYVY = 112
    -- cvtColor.YUV2RGBA_VYUY = 113
    -- cvtColor.YUV2BGRA_VYUY = 114
cvtColor.YUV2RGBA_Y422 = cvtColor.YUV2RGBA_UYVY
cvtColor.YUV2BGRA_Y422 = cvtColor.YUV2BGRA_UYVY
cvtColor.YUV2RGBA_UYNV = cvtColor.YUV2RGBA_UYVY
cvtColor.YUV2BGRA_UYNV = cvtColor.YUV2BGRA_UYVY

cvtColor.YUV2RGB_YUY2 = 115
cvtColor.YUV2BGR_YUY2 = 116
cvtColor.YUV2RGB_YVYU = 117
cvtColor.YUV2BGR_YVYU = 118
cvtColor.YUV2RGB_YUYV = cvtColor.YUV2RGB_YUY2
cvtColor.YUV2BGR_YUYV = cvtColor.YUV2BGR_YUY2
cvtColor.YUV2RGB_YUNV = cvtColor.YUV2RGB_YUY2
cvtColor.YUV2BGR_YUNV = cvtColor.YUV2BGR_YUY2

cvtColor.YUV2RGBA_YUY2 = 119
cvtColor.YUV2BGRA_YUY2 = 120
cvtColor.YUV2RGBA_YVYU = 121
cvtColor.YUV2BGRA_YVYU = 122
cvtColor.YUV2RGBA_YUYV = cvtColor.YUV2RGBA_YUY2
cvtColor.YUV2BGRA_YUYV = cvtColor.YUV2BGRA_YUY2
cvtColor.YUV2RGBA_YUNV = cvtColor.YUV2RGBA_YUY2
cvtColor.YUV2BGRA_YUNV = cvtColor.YUV2BGRA_YUY2

cvtColor.YUV2GRAY_UYVY = 123
cvtColor.YUV2GRAY_YUY2 = 124
    -- cvtColor.YUV2GRAY_VYUY = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_Y422 = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_UYNV = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_YVYU = cvtColor.YUV2GRAY_YUY2
cvtColor.YUV2GRAY_YUYV = cvtColor.YUV2GRAY_YUY2
cvtColor.YUV2GRAY_YUNV = cvtColor.YUV2GRAY_YUY2

    -- alpha premultiplication
cvtColor.RGBA2mRGBA = 125
cvtColor.mRGBA2RGBA = 126

cvtColor.RGB2YUV_I420 = 127
cvtColor.BGR2YUV_I420 = 128
cvtColor.RGB2YUV_IYUV = cvtColor.RGB2YUV_I420
cvtColor.BGR2YUV_IYUV = cvtColor.BGR2YUV_I420

cvtColor.RGBA2YUV_I420 = 129
cvtColor.BGRA2YUV_I420 = 130
cvtColor.RGBA2YUV_IYUV = cvtColor.RGBA2YUV_I420
cvtColor.BGRA2YUV_IYUV = cvtColor.BGRA2YUV_I420
cvtColor.RGB2YUV_YV12  = 131
cvtColor.BGR2YUV_YV12  = 132
cvtColor.RGBA2YUV_YV12 = 133
cvtColor.BGRA2YUV_YV12 = 134

cvtColor.COLORCVT_MAX  = 135

return cvtColor
