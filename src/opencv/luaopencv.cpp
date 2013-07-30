/* make several C wrappers to access opencv 2.4 c++ objects from lua */
/* extern "C" makes the functions callable from C */
#include "opencv2/features2d/features2d.hpp"

// function to sort the KeyPoints returned in DetectorExtractor
struct keyPointCompare {
  bool operator ()(const cv::KeyPoint & a, const cv::KeyPoint & b)
    const {return a.response>b.response;}
};

typedef struct KeyPointVector
{
  int length;
  cv::KeyPoint *data;
} KeyPointVector;

extern "C"

KeyPointVector detect(const CvMat* data, const char* detector_type )
{
  /* Mat img(height, width, CV_8UC3, pixels, step); */
  cv::Mat img(data);
  
  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(detector_type);
  std::vector<cv::KeyPoint> keypoints;
  detector->detect( img, keypoints );
  KeyPointVector res = {(int)keypoints.size(),keypoints.data()};
  return res;
}



// Sort the keypoints by value (See keyPointsCompare function in
// element/code/opencv.c) the keypoints which fall outside the mask
// have been given a low value.

// sort(keyPoints.begin(), keyPoints.end(), keyPointCompare());

// keyPoints.resize(foundPts);
// cout << "Found " << keyPoints.size() << " keypoints" << endl;


// printf("data (%d,%d) step: %d type: %d\n",
//   data->rows,data->cols,data->step,data->type);

// printf("img (%d,%d) type: %d\n",
//   img.rows,img.cols, img.type());

//   for(int i=0; i < keypoints.size(); i++){
//   printf("[%d] (%f, %f) %f\n",i,keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].response);
// }

//   //-- Draw keypoints
//   Mat img_keypoints;
//   drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1),
//     DrawMatchesFlags::DEFAULT );
//   //-- Show detected (drawn) keypoints
//   imshow("Keypoints", img_keypoints);

//   printf("[%d] img_key_points (%d,%d) type: %d\n",
//     (int)keypoints.size(),img_keypoints.rows,img_keypoints.cols,img_keypoints.type());
//   // hack we don't use class_id so override to pass the number of elements
