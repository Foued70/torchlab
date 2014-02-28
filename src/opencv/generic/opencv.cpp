#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/opencv.cpp"
#else

#include "opencv2/core/mat.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;

// from http://docs.opencv.org/modules/core/doc/basic_structures.html
// int depth    = mat->depth();
// CV_8U  - 8-bit unsigned integers ( 0..255 )                            torch.Byte
// CV_8S  - 8-bit signed integers ( -128..127 )                           torch.Char
// CV_16U - 16-bit unsigned integers ( 0..65535 )
// CV_16S - 16-bit signed integers ( -32768..32767 )
// CV_32S - 32-bit signed integers ( -2147483648..2147483647 )            torch.Int
// CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN ) torch.Float
// CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN ) torch.Double

// In order for the tensor to be useable in lua it has to be a
// userdata.  (This is being fixed in torch9 which is all based on
// ffi. You can get the cdata from the userdata (tensor declared in
// lua) but can't get the userdata back on the cdata. Or I didn't find
// a hack to get this to work.  So I have resigned myself to passing a
// THTensor* of a THTensor created in lua.

// **  TODO handle non-continous Mats ** 
// from http://stackoverflow.com/questions/11572156/stride-on-image-using-opencv-c

// The stride is that each new row of pixels in the cv::Mat doesn't
// start after the end of the last row. This is because the memory is
// faster if it starts on a multiple of 4bytes - so it only matters if
// the number of bytes in a row (number of pixels * number of colours)
// isn't a multiple of 4.

// You can check .isContinuous() will return true if there is no stride

// The safest way of accessing an image is to loop over all the rows
// and use .ptr(row) to get a pointer to the start of the row, then do
// any processing on that row.

// If you need to mix opencv with other libs you can create a cv::mat
// that will use your own memory for the data and you can tell opencv
// that there is no stride on this

extern "C" {
  
  void THTensor_(fromMat) (Mat* mat, THTensor* tensor)
  {

#if defined(TH_REAL_IS_BYTE)
    THArgCheck(mat->depth() == CV_8U  ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_CHAR)
    THArgCheck(mat->depth() == CV_8S  ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_SHORT)
    THArgCheck(mat->depth() == CV_16S ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_INT)
    THArgCheck(mat->depth() == CV_32S ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_LONG)
    THArgCheck(mat->depth() == -1     ,1, "OpenCV has no analog for Long");
#elif defined(TH_REAL_IS_FLOAT)
    THArgCheck(mat->depth() == CV_32F ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_DOUBLE)
    THArgCheck(mat->depth() == CV_64F ,1, "C don't do type conversions ");
#else
#error "Unknown type"
#endif

    THArgCheck(mat->isContinuous() == true ,1, "opencv mat is not continuous");

    int dims     = mat->dims;
    int channels = mat->channels();

    long nelem   = mat->total() * channels;

    real* mat_ptr = mat->ptr<real>(0);

    // make a storage for the 1D data
    THStorage* sdata =
      THStorage_(newWithData)(mat_ptr,nelem);

    THLongStorage* size   = THLongStorage_new();
    Mat_size(mat, size);

    THLongStorage* stride = THLongStorage_new();
    Mat_stride(mat, stride);

    THTensor_(setStorage)(tensor, sdata, 0, size, stride);

    THLongStorage_free(size);
    THLongStorage_free(stride);

  }

  Mat* THTensor_(toMat) (THTensor* tensor)
  {
    int i;
    Mat* mat;

#if defined(TH_REAL_IS_BYTE)
      int type = CV_8U;
#elif defined(TH_REAL_IS_CHAR)
    int type = CV_8S;
#elif defined(TH_REAL_IS_SHORT)
    int type = CV_16S;
#elif defined(TH_REAL_IS_INT)
    int type = CV_32S;
#elif defined(TH_REAL_IS_LONG)
    int type = CV_32S;
    THError("No analog for long in opencv please convert");
#elif defined(TH_REAL_IS_FLOAT)
    int type = CV_32F;
#elif defined(TH_REAL_IS_DOUBLE)
    int type = CV_64F;
#else
#error "Unknown type"
#endif

    if (!THTensor_(isContiguous)(tensor))
      THError("must pass contiguous tensor to opencv");

    real* data = THTensor_(data)(tensor);
    int ndims  = tensor->nDimension;
    if (ndims == 2) {
      int rows = tensor->size[0];
      int cols = tensor->size[1];
      mat = new Mat(rows,cols,type,data);
    } else if ((ndims == 3) && (tensor->size[2] <= 4 )) {
      int rows = tensor->size[0];
      int cols = tensor->size[1];
      int ctype = CV_MAKETYPE(type, tensor->size[2]);
      mat = new Mat(rows,cols,ctype,data);
    } else {
      int sizes[ndims];
      for(i=0;i<ndims;i++){
        sizes[i] = tensor->size[i];
      }
      mat = new Mat (ndims, sizes, type , data);
    }

    mat->addref(); // make sure the matrix sticks around

    return mat;
  }

  KeyPoint* THTensor_(toKeypoints) (THTensor* tensor)
  {
    int i;
    float x,y;

    if (!THTensor_(isContiguous)(tensor))
      THError("must pass contiguous tensor to opencv");
      
    int ndims = tensor->nDimension;
    if (ndims != 2)
      THError("must pass tensor dimensions of nx2");
    
    int rows = tensor->size[0];
    int cols = tensor->size[1];
    if (cols != 2)
      THError("must pass tensor dimensions of nx2");

    real* data = THTensor_(data)(tensor);
    
    KeyPoint * kpts = (KeyPoint *) malloc(rows * sizeof(KeyPoint));
    
    for (i = 0; i < rows; i++)
    {
      x = (float)data[i*2  ];
      y = (float)data[i*2+1];
      kpts[i] = KeyPoint(x, y, 1);
    }
    
    return kpts;
  }
  
} //extern "C"

#endif
