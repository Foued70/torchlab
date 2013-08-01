/* make several C wrappers to access opencv 2.4 c++ objects from lua */
/* extern "C" makes the functions callable from C */
#include <iostream>
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
  int detect(const CvMat* data, const char* detector_type, const CvMat* mask, KeyPoint* keypointsC, int npts) 
{
  /* Mat img(height, width, CV_8UC3, pixels, step); */
  Mat img(data);
  
  Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
  vector <KeyPoint> keypoints ;
  detector->detect( img, keypoints, mask);

  // Sort the keypoints by response.

  sort(keypoints.begin(), keypoints.end(), KeyPointCompare());

  if (npts > keypoints.size()) {
    npts = keypoints.size();
  }

  // memory at &keypoints[0] is guaranteed contiguous.
  memcpy(keypointsC,keypoints.data(),npts * sizeof(KeyPoint));

  return npts;
}

  void debug_keypoints(const CvMat* data, const KeyPoint* keyptr, int npts)
{
  vector<KeyPoint> keypoints (keyptr, keyptr + npts);
  Mat img(data);

  for(int i=0; i < keypoints.size(); i++){
    printf("[%d] (%d, %d) %f\n",i,
           (int)keypoints[i].pt.x, (int)keypoints[i].pt.y, 
           keypoints[i].response);
  }
  
  drawKeypoints( img, keypoints, img, Scalar::all(-1),
                 DrawMatchesFlags::DEFAULT );
  //Debug
  imshow("Keypoints", img);
  
}

} // extern "C"
