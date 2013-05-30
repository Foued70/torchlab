#include <TH.h>

void hough(THDoubleTensor *result, THDoubleTensor *image, long numR, long numA)
{  
  double *res = THDoubleTensor_data(result);
  double *img = THDoubleTensor_data(image);

  double numRows = image->size[1];
  double numCols = image->size[2];

  double maxRadius = sqrt(numRows * numRows + numCols * numCols) * 0.5;

}  


