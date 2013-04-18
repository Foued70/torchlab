#include <TH.h>

void rotate_by_quat(THDoubleTensor *result, 
                    THDoubleTensor *vectors, 
                    THDoubleTensor *quat)
{

  THDoubleTensor_resizeAs(result, vectors);
  
  double *res = THDoubleTensor_data(result);
  double *v   = THDoubleTensor_data(vectors);
  double *q   = THDoubleTensor_data(quat);
  
  double x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THDoubleTensor_nElement(vectors); i += stride)
    {
      x1 = q[1]*v[i+2] - q[2]*v[i+1];
      y1 = q[2]*v[i]   - q[0]*v[i+2];
      z1 = q[0]*v[i+1] - q[1]*v[i];
      
      res[i]   = v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
      res[i+1] = v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
      res[i+2] = v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
    }
}

void rotate_translate(THDoubleTensor *result, 
                      THDoubleTensor *vectors,
                      THDoubleTensor *trans,
                      THDoubleTensor *quat)
{

  THDoubleTensor_resizeAs(result, vectors);
  
  double *res = THDoubleTensor_data(result);
  double *v   = THDoubleTensor_data(vectors);
  double *q   = THDoubleTensor_data(quat);
  double *t   = THDoubleTensor_data(trans);
  
  double x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THDoubleTensor_nElement(vectors); i += stride)
    {
      x1 = q[1]*v[i+2] - q[2]*v[i+1];
      y1 = q[2]*v[i]   - q[0]*v[i+2];
      z1 = q[0]*v[i+1] - q[1]*v[i];
      
      res[i]   = t[0] + v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
      res[i+1] = t[1] + v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
      res[i+2] = t[2] + v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
    }
}

void translate_rotate(THDoubleTensor *result, 
                      THDoubleTensor *vectors,
                      THDoubleTensor *trans, 
                      THDoubleTensor *quat)
{

  THDoubleTensor_resizeAs(result, vectors);
  
  double *res = THDoubleTensor_data(result);
  double *v   = THDoubleTensor_data(vectors);
  double *q   = THDoubleTensor_data(quat);
  double *t   = THDoubleTensor_data(trans);
  
  double x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THDoubleTensor_nElement(vectors); i += stride)
    {
      res[i]   =   v[i] + t[0];
      res[i+1] = v[i+1] + t[1];
      res[i+2] = v[i+2] + t[2];

      x1 = q[1]*res[i+2] - q[2]*res[i+1];
      y1 = q[2]*res[i]   - q[0]*res[i+2];
      z1 = q[0]*res[i+1] - q[1]*res[i];
      
      res[i]   += 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
      res[i+1] += 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
      res[i+2] += 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
    }
}
