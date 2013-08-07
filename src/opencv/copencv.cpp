extern "C"
{
#include "TH.h"
}

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
  bool operator ()(const KeyPoint& a, const KeyPoint& b)
    const {return a.response>b.response;}
};

// return pointer to Mat. TODO Long storage.
extern "C"
{
  Mat* Mat_create (int height, int width, int type)
  {
    return new Mat (width, height, type);
  }
  // TODO: move these to a generic loop to handle all types.
  Mat* Mat_fromTensor(THDoubleTensor* tensor)
  {
    int i; 
    tensor = THDoubleTensor_newContiguous(tensor);
    double* data = THDoubleTensor_data(tensor);
    int ndims = tensor->nDimension;
    if (ndims == 2) {
      int rows = tensor->size[0];
      int cols = tensor->size[1];
      return new Mat(rows,cols,CV_64FC1,data);
    } else if ((ndims == 3) && (tensor->size[2] <= 4 )) {
      int rows = tensor->size[0];
      int cols = tensor->size[1];
      int type = CV_MAKETYPE(CV_64F, tensor->size[2]); 
      return new Mat(rows,cols,type,data);
    } else {
      int sizes[ndims];
      for(i=0;i<ndims;i++){
        sizes[i] = tensor->size[i];
      }
      return new Mat (ndims, sizes, CV_64F, data);
    }
  }
  // return a torch style THLongStorage for size of mat
  void Mat_size(Mat* mat, THLongStorage* size)
  {
    int i;
    int dims     = mat->dims;
    int channels = mat->channels();
    if (dims == 2) {
      if (channels == 1) {
        THLongStorage_resize(size,2);
      } else {
        THLongStorage_resize(size,3);
        THLongStorage_set(size,2,channels);
      }
      THLongStorage_set(size,0,mat->rows);
      THLongStorage_set(size,1,mat->cols);
    } else {
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
      THLongStorage_set(stride,dims,channels);
    } else {
      THLongStorage_resize(stride,dims);
    }
    for(i=0;i<mat->dims;i++){
      THLongStorage_set(stride,i,mat->step[i]/elemsize);
    }
  }

  // from http://docs.opencv.org/modules/core/doc/basic_structures.html
  // int depth    = mat->depth(); 
  // CV_8U  - 8-bit unsigned integers ( 0..255 )                            torch.Byte
  // CV_8S  - 8-bit signed integers ( -128..127 )                           torch.Char
  // CV_16U - 16-bit unsigned integers ( 0..65535 )
  // CV_16S - 16-bit signed integers ( -32768..32767 )
  // CV_32S - 32-bit signed integers ( -2147483648..2147483647 )            torch.Int
  // CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN ) torch.Float
  // CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN ) torch.Double

  // the userdata has to be set.  can get the cdata from the userdata
  // but can't get the userdata back on the cdata.
  void Mat_toTensor(Mat* mat, THDoubleTensor* tensor)
  {
    
    THArgCheck(mat->depth() == CV_64F      ,1, "don't do type conversions ");
    THArgCheck(mat->isContinuous() == true ,1, "opencv mat is not continuous");

    int dims     = mat->dims;
    int channels = mat->channels();
   
    // make a storage for the 1D data
    THDoubleStorage* sdata = 
      THDoubleStorage_newWithData((double*)mat->data,(long)(mat->total()));

    THLongStorage* size   = THLongStorage_new();
    Mat_size(mat, size);

    THLongStorage* stride = THLongStorage_new();
    Mat_stride(mat, stride);

    THDoubleTensor_setStorage(tensor, sdata, 0, size, stride);

    THLongStorage_free(size);
    THLongStorage_free(stride);

  }

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

  FeatureDetector* FeatureDetector_create(const char* detector_type)
  {
    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);

    return detector;
  }

  int detect(const Mat*  img,
             const char* detector_type,
             const Mat*  mask,
             KeyPoint*   keypointsC, int npts)
  {
    
    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);

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

} // extern "C"
