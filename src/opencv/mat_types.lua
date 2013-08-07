types = {}
types[0] = "CV_8U"
types[1] = "CV_8S"
types[2] = "CV_16U"
types[3] = "CV_16S"
types[4] = "CV_32S"
types[5] = "CV_32F"
types[6] = "CV_64F"

-- expanded macros from opencv2/core/types_c.h
types.CV_8U  = {}
types.CV_8S  = {}
types.CV_16U = {}
types.CV_16S = {}
types.CV_32S = {}
types.CV_32F = {}
types.CV_64F = {}
types.CV_USRTYPE1 = 7
types.CV_8U[1] = 0
types.CV_8U[2] = 8
types.CV_8U[3] = 16
types.CV_8U[4] = 24
types.CV_8S[1] = 1
types.CV_8S[2] = 9
types.CV_8S[3] = 17
types.CV_8S[4] = 25
types.CV_16U[1] = 2
types.CV_16U[2] = 10
types.CV_16U[3] = 18
types.CV_16U[4] = 26
types.CV_16S[1] = 3
types.CV_16S[2] = 11
types.CV_16S[3] = 19
types.CV_16S[4] = 27
types.CV_32S[1] = 4
types.CV_32S[2] = 12
types.CV_32S[3] = 20
types.CV_32S[4] = 28
types.CV_32F[1] = 5
types.CV_32F[2] = 13
types.CV_32F[3] = 21
types.CV_32F[4] = 29
types.CV_64F[1] = 6
types.CV_64F[2] = 14
types.CV_64F[3] = 22
types.CV_64F[4] = 30

return types
