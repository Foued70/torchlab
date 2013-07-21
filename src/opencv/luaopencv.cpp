/* make several C wrappers to access opencv 2.4 c++ objects from lua */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"


using namespace cv;
using namespace std;

extern "C"
void detectfeatures(const CvMat* data,char* detector_type)
{
  /* Mat img(height, width, CV_8UC3, pixels, step); */
  Mat img(data);
  
  Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
  vector<KeyPoint> keypoints1;
  detector->detect( img, keypoints1 );
}
