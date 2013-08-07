#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/copencv.cpp"
#else

#include "opencv2/core/mat.hpp"

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

// TODO: can't figure out why the Mat_(to)_THTensor expansion doesn't work...

extern "C" {
  
  void THTensor_(fromMat) (Mat* mat, THTensor* tensor)
  {

#if defined(TH_REAL_IS_BYTE)
    THArgCheck(mat->depth() == CV_8U      ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_CHAR)
    THArgCheck(mat->depth() == CV_8S      ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_SHORT)
    THArgCheck(mat->depth() == CV_16S      ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_INT)
    THArgCheck(mat->depth() == CV_32S      ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_LONG)
    THArgCheck(mat->depth() == -1          ,1, "OpenCV has no analog for Long");
#elif defined(TH_REAL_IS_FLOAT)
    THArgCheck(mat->depth() == CV_32F      ,1, "C don't do type conversions ");
#elif defined(TH_REAL_IS_DOUBLE)
    THArgCheck(mat->depth() == CV_64F      ,1, "C don't do type conversions ");
#else
#error "Unknown type"
#endif

    THArgCheck(mat->isContinuous() == true ,1, "opencv mat is not continuous");

    int dims     = mat->dims;
    int channels = mat->channels();

    // make a storage for the 1D data
    THStorage* sdata =
      THStorage_(newWithData)((real*)mat->data,(long)(mat->total()));

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
    THError("No analong for long in opencv please convert");
#elif defined(TH_REAL_IS_FLOAT)
    int type = CV_32F;
#elif defined(TH_REAL_IS_DOUBLE)
    int type = CV_64F;
#else
#error "Unknown type"
#endif

    tensor = THTensor_(newContiguous)(tensor);
    real* data = THTensor_(data)(tensor);
    int ndims = tensor->nDimension;
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
    // free the contiguous tensor wrapper (not data)
    THTensor_(free)(tensor);

    return mat;
  }
  
} //extern "C"

#endif
