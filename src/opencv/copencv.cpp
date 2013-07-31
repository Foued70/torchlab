/* make several C wrappers to access opencv 2.4 c++ objects from lua */
/* extern "C" makes the functions callable from C */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

// function to sort the KeyPoints returned in DetectorExtractor
struct KeyPointCompare {
  bool operator ()(const KeyPoint & a, const KeyPoint & b)
    const {return a.response>b.response;}
};

typedef struct KeyPointVector
{
  int length;
  KeyPoint *data;
} KeyPointVector;

extern "C" {
KeyPointVector detect(const CvMat* data, const char* detector_type, const CvMat* mask)
{
  /* Mat img(height, width, CV_8UC3, pixels, step); */
  Mat img(data);
  
  Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
  vector<KeyPoint> keypoints;
  detector->detect( img, keypoints );

  // Sort the keypoints by value. The keypoints which fall outside the mask
  // have been given a low value.

  sort(keypoints.begin(), keypoints.end(), KeyPointCompare());

  return {.length = (int)keypoints.size(), .data = keypoints.data()};
}

void draw_keypoints(const CvMat* data, const KeyPointVector kpv)
{
  vector<KeyPoint> keypoints (kpv.data, kpv.data + kpv.length);
  Mat img(data);

  for(int i=0; i < keypoints.size(); i++){
    printf("[%d] (%d, %d) %d\n",i,(int)keypoints[i].pt.x, (int)keypoints[i].pt.y, (int)keypoints[i].response);
  }
  
  drawKeypoints( img, keypoints, img, Scalar::all(-1),
                 DrawMatchesFlags::DEFAULT );
  //Debug
  imshow("Keypoints", img);
  
}

} // extern "C"
