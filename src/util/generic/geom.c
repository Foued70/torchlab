#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/geom.c"
#else

#include <TH.h>

void THTensor_(rotate_by_quat)(THTensor *result, 
                               THTensor *vectors, 
                               THTensor *quat)
{

  THTensor_(resizeAs)(result, vectors);
  
  real *res = THTensor_(data)(result);
  real *v   = THTensor_(data)(vectors);
  real *q   = THTensor_(data)(quat);
  
  real x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THTensor_(nElement)(vectors); i += stride)
    {
      x1 = q[1]*v[i+2] - q[2]*v[i+1];
      y1 = q[2]*v[i]   - q[0]*v[i+2];
      z1 = q[0]*v[i+1] - q[1]*v[i];
      
      res[i]   = v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
      res[i+1] = v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
      res[i+2] = v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
    }
}

void THTensor_(rotate_translate)(THTensor *result, 
                                 THTensor *vectors,
                                 THTensor *trans,
                                 THTensor *quat)
{

  THTensor_(resizeAs)(result, vectors);
  
  real *res = THTensor_(data)(result);
  real *v   = THTensor_(data)(vectors);
  real *q   = THTensor_(data)(quat);
  real *t   = THTensor_(data)(trans);
  
  real x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THTensor_(nElement)(vectors); i += stride)
    {
      x1 = q[1]*v[i+2] - q[2]*v[i+1];
      y1 = q[2]*v[i]   - q[0]*v[i+2];
      z1 = q[0]*v[i+1] - q[1]*v[i];
      
      res[i]   = t[0] + v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
      res[i+1] = t[1] + v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
      res[i+2] = t[2] + v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
    }
}

void THTensor_(translate_rotate)(THTensor *result, 
                                 THTensor *vectors,
                                 THTensor *trans, 
                                 THTensor *quat)
{

  THTensor_(resizeAs)(result, vectors);
  
  real *res = THTensor_(data)(result);
  real *v   = THTensor_(data)(vectors);
  real *q   = THTensor_(data)(quat);
  real *t   = THTensor_(data)(trans);
  
  real x1, y1, z1;
  long i;
  long stride = vectors->stride[0];
  /* Allows function to accept a 3 element vector rather than resize to 1x3 */
  if (vectors->nDimension == 1){
    stride = vectors->size[0];
  } 
#pragma omp parallel for private(i,x1,y1,z1)
  for (i = 0; i < THTensor_(nElement)(vectors); i += stride)
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

#endif
