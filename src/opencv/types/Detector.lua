types = {}

types["FAST"]       = 1   -- FastFeatureDetector
types["STAR"]       = 2   -- StarFeatureDetector
types["SIFT"]       = 3   -- SIFT (nonfree module)
types["SURF"]       = 4   -- SURF (nonfree module)
types["ORB"]        = 5   -- ORB
types["BRISK"]      = 6   -- BRISK
types["MSER"]       = 7   -- MSER
types["GFTT"]       = 8   -- GoodFeaturesToTrackDetector
types["HARRIS"]     = 9   -- GoodFeaturesToTrackDetector with Harris detector enabled
types["Dense"]      = 10  -- DenseFeatureDetector
types["SimpleBlob"] = 11  -- SimpleBlobDetector

return types
