#include <TH.h>

void rotate_by_quat(THDoubleTensor *result,
                    THDoubleTensor *quat,
                    THDoubleTensor *vect
                    )
{
  long outDimension      = quat->nDimension + vect->nDimension -1;
  THLongStorage *newSize = THLongStorage_newWithSize(outDimension);
  long *sd               = THLongStorage_data(newSize);

  long offset         = 0;
  // which dimension contains quat or vect
  long quatDim        = quat->nDimension-1;
  long vectDim        = vect->nDimension-1;
  long quatSize       = quat->size[quatDim]; // == 4
  long vectSize       = vect->size[vectDim]; // == 3 or 4
  long nElementQuat   = THDoubleTensor_nElement(quat);
  long nElementVect   = THDoubleTensor_nElement(vect);
  long quatDimStride  = 1;
  long vectDimStride  = 1;
  long quatElemStride = quatSize;
  long vectElemStride = vectSize;
  long quatVectStride = quatSize;
  long vectVectStride = vectSize;

  long i,j;

  char DHW = 0;

  // quaternions and vectors are either Nx3,4 oer 3,4 x N but must be consistent.
  if ((quatSize != 4) || ((vectSize != 3) && vectSize != 4)) {
    vectDim = 0;
    quatDim = 0;
    quatSize = quat->size[vectDim];
    vectSize = vect->size[quatDim];
    quatElemStride = 1;
    vectElemStride = 1;
    quatDimStride  = quat->stride[vectDim];
    vectDimStride  = quat->stride[quatDim];
    DHW = 1;
  }

  THArgCheck(quatSize == 4,                         2,
             "quaternion is a vector of length 4");
  THArgCheck(((vectSize == 3) || (vectSize == 4)),3,
             "point vectors should be of length 3 or 4");

  long n_vect = nElementVect / vectSize;
  long n_quat = nElementQuat / quatSize;

  long start    = 0;
  long quat_end = quat->nDimension-1;
  long vect_end = vect->nDimension-1;
  if (DHW > 0) {
    // printf("Doing DHW\n");
    start++;
    quat_end++;
    vect_end++;
    sd[offset] = vectSize;
    offset += 1;
  }

  for (i = start ; i < quat_end ; i++){
    sd[offset] = quat->size[i];
    offset += 1;
  }

  for (i = start ; i < vect_end ; i++){
    sd[offset] = vect->size[i];
    offset += 1;
  }
  if (DHW==0) {
    sd[offset] = vectSize;
  }

  THDoubleTensor_resize(result, newSize, NULL);
  if (vectSize == 4) // incase homogenous coordinates are requested
    THDoubleTensor_fill(result,1);
  THLongStorage_free(newSize);

  double *res = THDoubleTensor_data(result);
  double *q   = THDoubleTensor_data(quat);
  double *v   = THDoubleTensor_data(vect);

  double x1, y1, z1;
  long resDimStride  = result->stride[outDimension-1];
  long resElemStride = vectSize;

  if (DHW>0) {
    resDimStride  = result->stride[0];
    resElemStride = result->stride[outDimension-1];
  }

  /* printf("quatSize: %ld; vectSize: %ld\n",quatSize, vectSize); */
  /* printf("quatDimStride: %ld; vectDimStride: %ld; resDimStride: %ld\n", */
  /*        quatDimStride, vectDimStride,resDimStride); */
  /* printf("quatElemStride: %ld; vectElemStride: %ld; resElemStride: %ld\n", */
  /*        quatElemStride, vectElemStride,resElemStride); */

  double * res0 = res;
  double * res1 = res0 + resDimStride;
  double * res2 = res1 + resDimStride;

  double * q0 = q;
  double * q1 = q0+quatDimStride;
  double * q2 = q1+quatDimStride;
  double * q3 = q2+quatDimStride;
  for (j = 0; j < n_quat; j++)
    {
      double * v0 = v;
      double * v1 = v0+vectDimStride;
      double * v2 = v1+vectDimStride;
#pragma omp parallel for private(i,x1,y1,z1)
      for (i = 0; i < n_vect; i++)
        {
          x1 = (*q1)*(*v2) - (*q2)*(*v1);
          y1 = (*q2)*(*v0) - (*q0)*(*v2);
          z1 = (*q0)*(*v1) - (*q1)*(*v0);

          (*res0) = (*v0) + 2 * ((*q3)*x1 + (*q1)*z1 - (*q2)*y1);
          (*res1) = (*v1) + 2 * ((*q3)*y1 + (*q2)*x1 - (*q0)*z1);
          (*res2) = (*v2) + 2 * ((*q3)*z1 + (*q0)*y1 - (*q1)*x1);

          v0+=vectElemStride; v1+=vectElemStride; v2+=vectElemStride;
          res0+=resElemStride; res1+=resElemStride; res2+=resElemStride;
        }
      q0+=quatElemStride; q1+=quatElemStride; q2+=quatElemStride; q3+=quatElemStride;
    }
}

void rotate_translate(THDoubleTensor *result,
                      THDoubleTensor *quat,
                      THDoubleTensor *trans,
                      THDoubleTensor *vect
                      )
{
  long outDimension      = quat->nDimension + vect->nDimension -1;
  THLongStorage *newSize = THLongStorage_newWithSize(outDimension);
  long *sd               = THLongStorage_data(newSize);

  long offset = 0;
  long quatStride   = quat->size[quat->nDimension-1];
  long transStride  = trans->size[trans->nDimension-1];
  long vectStride   = vect->size[vect->nDimension-1];
  long nElementQuat = THDoubleTensor_nElement(quat);
  long nElementVect = THDoubleTensor_nElement(vect);
  long nQuat        = nElementQuat / quatStride;
  long nTrans       = THDoubleTensor_nElement(trans) / transStride;
  long i,j;

  THArgCheck(quatStride == 4,                           2,
             "quaternion is a vector of length 4");
  THArgCheck(nTrans == nQuat,                           3,
             "Different number of translations and rotations");
  THArgCheck(((transStride == 3) || (transStride == 4)),3,
             "translation vectors should be of length 3 or 4");
  THArgCheck(((vectStride == 3) || (vectStride == 4)),  4,
             "point vectors should be of length 3 or 4");

  for (i = 0 ; i < quat->nDimension-1 ; i++){
    sd[offset] = quat->size[i];
    offset += 1;
  }

  for (i = 0 ; i < vect->nDimension-1 ; i++){
    sd[offset] = vect->size[i];
    offset += 1;
  }

  sd[offset] = vectStride;

  THDoubleTensor_resize(result, newSize, NULL);
  if (vectStride == 4) // incase homogenous coordinates are requested
    THDoubleTensor_fill(result,1);
  THLongStorage_free(newSize);

  double *res = THDoubleTensor_data(result);
  double *q   = THDoubleTensor_data(quat);
  double *t   = THDoubleTensor_data(trans);
  double *v   = THDoubleTensor_data(vect);

  double x1, y1, z1;


  for (j = 0; j < nElementQuat; j += quatStride)
    {
#pragma omp parallel for private(i,x1,y1,z1)
      for (i = 0; i < nElementVect; i += vectStride)
        {
          x1 = q[1]*v[i+2] - q[2]*v[i+1];
          y1 = q[2]*v[i]   - q[0]*v[i+2];
          z1 = q[0]*v[i+1] - q[1]*v[i];

          res[i]   = t[0] + v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
          res[i+1] = t[1] + v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
          res[i+2] = t[2] + v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);
        }
      q   += quatStride;
      t   += transStride;
      res += nElementVect;
    }
}

void translate_rotate(THDoubleTensor *result,
                      THDoubleTensor *trans,
                      THDoubleTensor *quat,
                      THDoubleTensor *vect
                      )
{
  long outDimension      = quat->nDimension + vect->nDimension -1;
  THLongStorage *newSize = THLongStorage_newWithSize(outDimension);
  long *sd               = THLongStorage_data(newSize);

  long offset = 0;
  long quatStride   = quat->size[quat->nDimension-1];
  long transStride  = trans->size[trans->nDimension-1];
  long vectStride   = vect->size[vect->nDimension-1];
  long nElementQuat = THDoubleTensor_nElement(quat);
  long nElementVect = THDoubleTensor_nElement(vect);
  long nQuat        = nElementQuat / quatStride;
  long nTrans       = THDoubleTensor_nElement(trans) / transStride;

  long i,j;

  THArgCheck(nTrans == nQuat,                           2,
             "Different number of translations and rotations");
  THArgCheck(((transStride == 3) || (transStride == 4)),2,
             "translation vectors should be of length 3 or 4");
  THArgCheck(quatStride == 4,                           3,
             "quaternion is a vector of length 4");
  THArgCheck(((vectStride == 3) || (vectStride == 4)),  4,
             "point vectors should be of length 3 or 4");

  for (i = 0 ; i < quat->nDimension-1 ; i++){
    sd[offset] = quat->size[i];
    offset += 1;
  }

  for (i = 0 ; i < vect->nDimension-1 ; i++){
    sd[offset] = vect->size[i];
    offset += 1;
  }

  sd[offset] = vectStride;

  THDoubleTensor_resize(result, newSize, NULL);
  if (vectStride == 4) // incase homogenous coordinates are requested
    THDoubleTensor_fill(result,1);
  THLongStorage_free(newSize);

  double *res = THDoubleTensor_data(result);
  double *q   = THDoubleTensor_data(quat);
  double *t   = THDoubleTensor_data(trans);
  double *v   = THDoubleTensor_data(vect);

  double x1, y1, z1;


  for (j = 0; j < nElementQuat; j += quatStride)
    {
#pragma omp parallel for private(i,x1,y1,z1)
      for (i = 0; i < nElementVect; i += vectStride)
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
      q   += quatStride;
      t   += transStride;
      res += nElementVect;
    }
}
