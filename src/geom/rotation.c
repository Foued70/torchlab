#include <TH.h>

void rotate_by_quat(THDoubleTensor *result,
                    THDoubleTensor *quat,
                    THDoubleTensor *vect
                    )
{
  long outDimension      = quat->nDimension + vect->nDimension -1;
  THLongStorage *newSize = THLongStorage_newWithSize(outDimension);
  long *sd               = THLongStorage_data(newSize);

  long offset = 0;
  long quatStride = quat->size[quat->nDimension-1];
  long vectStride = vect->size[vect->nDimension-1];
  long nElementQuat      = THDoubleTensor_nElement(quat);
  long nElementVect      = THDoubleTensor_nElement(vect);
  long i,j;

  THArgCheck(quatStride == 4,                         2,
             "quaternion is a vector of length 4");
  THArgCheck(((vectStride == 3) || (vectStride == 4)),3,
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

          res[i]   = v[i]   + 2 * (q[3]*x1 + q[1]*z1 - q[2]*y1);
          res[i+1] = v[i+1] + 2 * (q[3]*y1 + q[2]*x1 - q[0]*z1);
          res[i+2] = v[i+2] + 2 * (q[3]*z1 + q[0]*y1 - q[1]*x1);

        }
      q   += quatStride;
      res += nElementVect;
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

