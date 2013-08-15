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

using namespace cv;
using namespace std;

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

  void FeatureDetector_destroy(FeatureDetector* detector)
  {
    delete(detector);
  }

  void debug_keypoints(Mat* img, const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);

    cout << "type: " << img->type() << endl;
    for(int i=0; i < keypoints.size(); i++){
      printf("[%d] (%d, %d) %f\n",i,
             (int)keypoints[i].pt.x, (int)keypoints[i].pt.y,
             keypoints[i].response);
    }

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

} // extern "C"
