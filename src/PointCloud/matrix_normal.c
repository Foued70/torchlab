#include <TH.h>
#include <THTensor.h>
#include "generic/THTensorMath.h"
#include <stdio.h>

void cross_product(THDoubleTensor *result,
                    THDoubleTensor *m1,
                    THDoubleTensor *m2,
                    int dim
                    )
{
 if(THDoubleTensor_nDimension(m1) != THDoubleTensor_nDimension(m2))
    printf("%d  %d", THDoubleTensor_nDimension(m1), THDoubleTensor_nDimension(m2));
 THDoubleTensor_cross(result,m1,m2,dim);
}