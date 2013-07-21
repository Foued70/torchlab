/* make several C wrappers to access opencv 2.4 c++ objects from lua */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"


using namespace cv;
using namespace std;

extern "C"
CvMat detectfeatures(const CvMat* data,const char* detector_type)
{
  /* Mat img(height, width, CV_8UC3, pixels, step); */
  Mat img(data);
  
  Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
  vector<KeyPoint> keypoints;
  detector->detect( img, keypoints );

  //-- Draw keypoints
  Mat img_keypoints;
  drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  //-- Show detected (drawn) keypoints
  imshow("Keypoints", img_keypoints);

  /* convert vector<KeyPoint> to CvMat */
  vector<Point2f> points;
  KeyPoint::convert(keypoints,points);
  CvMat mat_points = Mat(points);
  
  return mat_points;
}
