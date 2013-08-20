/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

void printParams( Algorithm* algorithm ) {
  vector<string> parameters;
  string param, helpText, typeText;
  int type;

  algorithm->getParams(parameters);
  cout << "Found " << parameters.size() << " parameters" << endl;
  for (int i = 0; i < (int) parameters.size(); i++) {
    param    = parameters[i];
    type     = algorithm->paramType(param);

    cout << " + '" << param << "' ";
    switch (type) {
    case Param::BOOLEAN:
      typeText = "bool";
      cout << "(" << typeText << ") " << algorithm->get<bool>(param);
      break;
    case Param::INT:
      typeText = "int";
      cout << "(" << typeText << ") " << algorithm->get<int>(param);
      break;
    case Param::REAL:
      typeText = "double";
      cout << "(" << typeText << ") " << algorithm->get<double>(param);
      break;
    case Param::STRING:
      typeText = "string";
      cout << "(" << typeText << ") " << algorithm->get<string>(param);
      break;
    case Param::MAT:
      typeText = "Mat";
      cout << "(" << typeText << ") " << algorithm->get<Mat>(param);
      break;
    case Param::ALGORITHM:
      typeText = "Algorithm";
      cout << "(" << typeText << ") " << algorithm->get<Algorithm>(param);
      break;
    case Param::MAT_VECTOR:
      typeText = "Mat vector";
      cout << "(" << typeText << ") " ; // << algorithm->get<vector<Mat> >(param);
      break;
    case Param::FLOAT:
      typeText = "float";
      cout << "(" << typeText << ") " << algorithm->getDouble(param);
      break;
    case Param::UNSIGNED_INT:
      typeText = "unsigned int";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
    case Param::UINT64:
      typeText = "uint64";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
    case Param::UCHAR:
      typeText = "uchar";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
    }
    // string paramHelp = algorithm->paramHelp(param);
    // cout << paramHelp;
    cout << endl;
  }
}

extern "C"
{
  // return a torch style THLongStorage for size of mat
  void Mat_size(Mat* mat, THLongStorage* size)
  {
    int i;
    int dims     = mat->dims;
    int channels = mat->channels();
    if (channels == 1) {
      THLongStorage_resize(size,dims);
    } else {
      THLongStorage_resize(size,dims+1);
      THLongStorage_set(size,dims,channels);
    }
    for (i=0;i<dims;i++){
      THLongStorage_set(size,i,mat->size[i]);
    }
  }

  int  Mat_size0(Mat *mat) {
    return mat->size[0];
  }

  int  Mat_size1(Mat *mat) {
    return mat->size[1];
  }

  int  Mat_size2(Mat *mat) {
    return mat->size[2];
  }
  // return a torch style THLongStorage for stride of mat
  void Mat_stride(Mat* mat, THLongStorage* stride)
  {
    int i;
    int dims     = mat->dims;
    int channels = mat->channels();
    int elemsize = mat->elemSize1();

    if (channels > 1) {
      THLongStorage_resize(stride,dims+1);
      THLongStorage_set(stride,dims,1);
    } else {
      THLongStorage_resize(stride,dims);
    }
    for(i=0;i<mat->dims;i++){
      THLongStorage_set(stride,i,mat->step[i]/elemsize);
    }
  }
}

// toTensor and fromTensor code
#include "generic/copencv.cpp"
#include "THGenerateAllTypes.h"

extern "C" {
  Mat* Mat_create (int height, int width, int type)
  {
    return new Mat (width, height, type);
  }

  Mat* Mat_clone (Mat* mat)
  {
    // need to make a copy like mat->clone() but dynamic memory allocation.
    Mat * clone = new Mat(mat->dims,mat->size[0],mat->type());
    mat->copyTo(*clone);
    return clone;
  }
  // TODO move to image processing
  void Mat_convert(Mat* input, Mat* output, int cvt_type)
  {
    cvtColor(*input, *output, cvt_type);
  }

  // convert from int to enum in string form for enum < 31
  string enum_strings[] = {
    "CV_8UC1", "CV_8SC1", "CV_16UC1", "CV_16SC1", "CV_32SC1", "CV_32FC1", "CV_64FC1", "CV_USRTYPE1",
    "CV_8UC2", "CV_8SC2", "CV_16UC2", "CV_16SC2", "CV_32SC2", "CV_32FC2", "CV_64FC2", "CV_USRTYPE2",
    "CV_8UC3", "CV_8SC3", "CV_16UC3", "CV_16SC3", "CV_32SC3", "CV_32FC3", "CV_64FC3", "CV_USRTYPE3",
    "CV_8UC4", "CV_8SC4", "CV_16UC4", "CV_16SC4", "CV_32SC4", "CV_32FC4", "CV_64FC4", "CV_USRTYPE4"};

  int Mat_depth(Mat* mat)
  {
    return mat->depth();
  }

  void Mat_info(Mat* mat)
  {
    int i;
    cout << "dims:        " << mat->dims       << endl;
    cout << "channels:    " << mat->channels() << endl;
    cout << "depth:       " << enum_strings[mat->depth()] << endl;
    cout << "type:        " << enum_strings[mat->type()]  << endl;
    cout << "elemsize:    " << mat->elemSize()  << endl;
    cout << "elemsize1:   " << mat->elemSize1() << endl;
    cout << "total:       " << mat->total()     << endl;
    cout << "contiguous?: " << mat->isContinuous() << endl;
    if (mat->dims == 2) {
      cout << "rows:      " << mat->rows       << endl;
      cout << "cols:      " << mat->cols       << endl;
    }
    for (i=0;i<mat->dims;i++){
      cout << "size["<<i<<"]:   " << mat->size[i]   << endl;
    }
    for (i=0;i<mat->dims;i++){
      cout << "step["<<i<<"]:   " << mat->step[i]   << endl;
    }

  }

  Mat* Mat_loadImage(const char* fname)
  {
    // make sure that the pointer is dynamically allocated
    return new Mat(imread(fname));
  }

  // this is primarily for debugging as it interferes w/ luvit
  void Mat_showImage(Mat* mat, const char* wname)
  {
    // Create a window for display.
    namedWindow(wname, CV_WINDOW_AUTOSIZE );
    imshow(wname, *mat);
  }

  void Mat_destroy(Mat* mat)
  {
    delete(mat);
  }


  // -----------------------------
  // FeatureDetector
  // -----------------------------

  // function to sort the KeyPoints returned in DetectorExtractor
  struct KeyPointCompare {
    bool operator ()(const KeyPoint& a, const KeyPoint& b)
      const {return a.response>b.response;}
  };

  FeatureDetector* FeatureDetector_create(const char* detector_type)
  {
    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
    detector.addref(); // make sure the Ptr stays around TODO: check memleak
    return detector;
  }

  void FeatureDetector_parameters(FeatureDetector* detector)
  {
    printParams((Algorithm*)detector);
  }

  int FeatureDetector_detect(FeatureDetector* detector,
                             const Mat*  img,
                             const Mat*  mask,
                             KeyPoint*   keypointsC,
                             int npts)
  {

    vector <KeyPoint> keypoints ;
    detector->detect( *img, keypoints, *mask);

    // Sort the keypoints by response.
    sort(keypoints.begin(), keypoints.end(), KeyPointCompare());

    if (npts > keypoints.size()) {
      npts = keypoints.size();
    }

    // memory at &keypoints[0] is guaranteed contiguous.
    memcpy(keypointsC,keypoints.data(),npts * sizeof(KeyPoint));

    return npts;
  }

  //return matrix representing the harris response at each point in src
  //note that FeatureDetector_detect with harris only returns the points, not the response!
  Mat* detectCornerHarris(Mat* src, int blockSize, int ksize, int k)
  {
    Mat returnMat = Mat::zeros( src->size(), CV_64FC1 );
    cornerHarris(*src, returnMat, blockSize, ksize, k);
    return new Mat(returnMat);
  }

  //return a matrix instead of a keypoint list, representing x,y,response
  Mat* getMatFromKeypoints(const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);

    vector<Point3d> points;

    for(int i=0; i < keypoints.size(); i++){
      points.push_back(Point3d(keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].response));
    }
    return new Mat(points);
  }

  void FeatureDetector_destroy(FeatureDetector* detector)
  {
    // calling release() on the Ptr (rather than deleting it) calls the correct destructor for the object.
    ((Ptr<FeatureDetector>)detector).release();
  }

  void dump_keypoints(const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);

    for(int i=0; i < keypoints.size(); i++){
      printf("[%d] (%d, %d) %f\n",i,
             (int)keypoints[i].pt.x, (int)keypoints[i].pt.y,
             keypoints[i].response);
    }
  }

  void draw_keypoints(Mat* img, const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);
    drawKeypoints( *img, keypoints, *img, Scalar::all(-1),
                   DrawMatchesFlags::DEFAULT );
    //Debug
    imshow("Keypoints", *img);
  }

  DescriptorExtractor* DescriptorExtractor_create(const char* feature_type)
  {
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(feature_type);
    extractor.addref(); // make sure the Ptr stays around TODO: check memleak
    return extractor;
  }

  void DescriptorExtractor_compute(DescriptorExtractor* extractor,
                                   const Mat*  img,
                                   Mat*  descriptor,
                                   KeyPoint*   keypointsC,
                                   int npts)
  {

    vector <KeyPoint> keypoints(npts);
    memcpy(keypoints.data(),keypointsC,npts * sizeof(KeyPoint));

    extractor->compute(*img, keypoints, *descriptor);

  }

  void DescriptorExtractor_destroy(DescriptorExtractor* extractor)
  {
    delete(extractor);
  }

  // -----------------------------
  // Edge detection
  // -----------------------------
  Mat* CannyDetectEdges(Mat* src, double threshold1, double threshold2)
  {
    Mat* dest = new Mat(0,0,src->type());
    Canny(*src, *dest, threshold1, threshold2);
    return dest;
  }

  // -----------------------------
  // Hough transform
  // -----------------------------

  Mat* HoughLinesRegular(Mat* image, double rho, double theta, int threshold, double srn, double stn)
  {
    Mat* lines = new Mat(0,0,image->type());;
    HoughLines(*image, *lines, rho, theta, threshold, srn, stn);
    return lines;
  }

  Mat* HoughLinesProbabilistic(Mat* image,double rho, double theta, int threshold, double minLineLength, double maxLineGap) 
  {
    Mat* lines = new Mat(0,0,image->type());
    HoughLinesP(*image, *lines, rho, theta, threshold, minLineLength, maxLineGap);
    return lines;
  }

  // -----------------------------
  // Erosion and dilation
  // -----------------------------

  Mat* getStructuringElement(int type, int size_x, int size_y, int center_x, int center_y)
  {
    return new Mat(getStructuringElement(type,
                                         Size(size_x, size_y),
                                         Point(center_x, center_y) ));
  }
  void dilate(Mat*  src, Mat* structuringElement)
  {
    dilate(*src, *src, *structuringElement);
  }

  void erode(Mat*  src, Mat* structuringElement)
  {
    erode(*src, *src, *structuringElement);
  }


  // -----------------------------
  // Matcher
  // -----------------------------
  DescriptorMatcher* DescriptorMatcher_create(const char* feature_type)
  {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(feature_type);
    matcher.addref(); // make sure the Ptr stays around TODO: check memleak
    return matcher;
  }

  int DescriptorMatcher_match(DescriptorMatcher* matcher,
                              const Mat*  descriptors_src,
                              const Mat*  descriptors_dest,
                              DMatch*   matchesC,
                              int npts)
  {
    vector<DMatch> matches;

    matcher->match(*descriptors_src, *descriptors_dest, matches);

    int nmatches = matches.size();

    memcpy(matchesC, matches.data(),nmatches * sizeof(DMatch));

    return nmatches;
  }

  void DescriptorMatcher_destroy(DescriptorMatcher* matcher)
  {
    delete(matcher);
  }

  int DescriptorMatcher_reduceMatches(DMatch*   matchesptr, int nmatches, DMatch*   matchesReducedC)
  {
    vector<DMatch> matches (matchesptr, matchesptr + nmatches);
    double max_dist = 0, min_dist = 100000000;
    for( int i = 0; i < nmatches; i++ )
      {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

    //select only good matches
    vector< DMatch > good_matches;

    for( int i = 0; i < nmatches; i++ )
      {
        if( matches[i].distance < 3*min_dist )
          {
            good_matches.push_back( matches[i]);
          }
      }

    memcpy(matchesReducedC,good_matches.data(),good_matches.size() * sizeof(DMatch));

    return good_matches.size();

  }

  // -----------------------------
  // ImageProcessing
  // -----------------------------
  Mat* getHomography(const KeyPoint* keyptr_src, int npts_src,
                     const KeyPoint* keyptr_dest, int npts_dest,
                     const DMatch* matchptr, int npts_match)
  {
    vector<KeyPoint> keypoints_src (keyptr_src, keyptr_src + npts_src);
    vector<KeyPoint> keypoints_dest (keyptr_dest, keyptr_dest + npts_dest);

    vector<DMatch> matches (matchptr, matchptr + npts_match);

    //-- Localize the object
    vector<Point2d> src;
    vector<Point2d> dest;

    for( int i = 0; i < npts_match; i++ )
      {
        //-- Get the keypoints from the good matches
        src.push_back( keypoints_src[ matches[i].queryIdx ].pt );
        dest.push_back( keypoints_dest[ matches[i].trainIdx ].pt );
      }
    Mat H = findHomography( src, dest, CV_RANSAC );
    H.convertTo(H,CV_32FC1,1,0);

    return new Mat(H);
  }

  Mat* warpImage(const Mat* src, const Mat* transform)
  {
    Mat* warpedSrc= new Mat(0,0,src->type());
    warpPerspective(*src, *warpedSrc, *transform, Size(src->cols*2, src->rows*2));
    return warpedSrc;
  }

  Mat* combineImages(const Mat* src,
                     const Mat* dest,
                     const Mat* transform,
                     int result_size_x,
                     int result_size_y,
                     int result_center_x,
                     int result_center_y)
  {
    //x-form accumulator;
    Mat Acc = (Mat_<double>(3, 3) << 1, 0, result_center_x, 0, 1, result_center_y, 0, 0, 1);
    Mat result;
    result = Mat::zeros(result_size_x, result_size_y, dest->type());
    warpPerspective( *dest, result, Acc, result.size() );

    //accumulate transformation
    Acc = Acc * *transform;

    //save target
    Mat tempresult = result.clone();

    warpPerspective( *src, result, Acc, result.size() );
    result = result + tempresult;

    return new Mat(result);
  }

} // extern "C"
