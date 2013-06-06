#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/hough.c"
#else

/*======================================================================
 * File: hough
 *
 * Description: hough Transform
 *
 * Created: May 27, 2013, 12:53 PM
 *
 *
 * Author: Sonia Jin
 *======================================================================*/

#include <luaT.h>
#include <TH.h>

/***
    double get_window_average(double *hT, int r, int c, int winr, int winc, int numR, int numC){

    double ret = 0.0;
    double cnt = 0.0;

    double *tmpPt;

    for (int i = r-winr; i < r+winr; i++){
    for (int j = c-winc; j < c+winc; j++){
    int ir = fmin(fmax(0, i), numR-1);
    int jc = j % numC;
    tmpPt = hT + (ir * numC) + jc;
    ret += *tmpPt;
    cnt ++;
    }
    }
    ret = ret/cnt;
    return ret;
    }

    void local_contrast_normalization(double *hT, int numRadius, int numAngles, int filtsize)
    {
    double hTOrigArr[numRadius][numAngles];
    double *hTOrig = &hTOrigArr[0][0];

    memcpy(hTOrig, hT, numRadius * numAngles * sizeof(double));
    double *tmpHTNPt;
    double *tmpHTOPt;

    for (int nA = 0; nA < numAngles; nA ++){
    for (int nR = 0; nR < numRadius; nR ++){
    tmpHTNPt = hT + (nR * numAngles) + nA;
    tmpHTOPt = hTOrig + (nR * numAngles) + nA;
    double avg;
    if (numAngles % 8 == 0 && nA % (int)(numAngles/8) == 0){
    avg = get_window_average(hTOrig, nR, nA, 1, 1, numRadius, numAngles);
    *tmpHTNPt = avg;
    *tmpHTOPt = avg;
    }
    if(*tmpHTNPt > 0){
    avg = get_window_average(hTOrig, nR, nA, filtsize, filtsize, numRadius, numAngles);
    if (*tmpHTNPt <= avg) {
    *tmpHTNPt = 0;
    }
    else {
    *tmpHTNPt = *tmpHTNPt - avg;
    }
    }
    }
    }

    }


    void hough(THDoubleTensor *result, THDoubleTensor *image)
    {
    double *res = THDoubleTensor_data(result);
    double *img = THDoubleTensor_data(image);

    long numRows = (long)(image->size[1]);
    long numCols = (long)(image->size[2]);

    long numRadius = (long)(result->size[1]);
    long numAngles = (long)(result->size[2]);

    double maxRadius = sqrt(numRows * numRows + numCols * numCols) * 0.5;

    double incAngles = 2 * M_PI / numAngles;
    double incRadius = maxRadius / (numRadius - 1);

    double maxImageVal = THDoubleTensor_maxall(image);

    for (int row = 0; row < numRows; row ++)
    {
    for (int col = 0; col < numCols; col ++)
    {
    //val = img->row, col
    double *tmpImgPt = img + (row * numCols) + col;
    double val = *tmpImgPt;

    if (val > 0.05 * maxImageVal)
    {
    //increment vote
    for (int nA = 0; nA < numAngles; nA ++)
    {
    double angle = nA * incAngles;
    double radiu = sin(angle) * (row-numRows/2) + cos(angle) * (col-numCols/2);
    double nRR = radiu/incRadius;
    int nR = (int)(nRR);

    if (nRR >=0 && nRR < numRadius)
    {
    double *tmpHoughPt = res + (nR * numAngles) + nA;
    *tmpHoughPt += val;

    }
    }
    }
    }
    }

    int filtsize = 5;
    local_contrast_normalization(res, numRadius, numAngles, filtsize);
    }

    static int libhough_(Main_houghTransform)(lua_State *L){
    THDoubleTensor *result = luaT_checkudata(L, 1, torch_Tensor);
    THDoubleTensor *image  = luaT_checkudata(L, 2, torch_Tensor);

    hough(result, image);
    }
***/

static int libhough_(Main_houghTransform)(lua_State *L){
  THTensor *result = luaT_checkudata(L, 1, torch_Tensor);
  THTensor *image  = luaT_checkudata(L, 2, torch_Tensor);

  real *res = THTensor_(data)(result);
  real *img = THTensor_(data)(image);

  long numRows = (long)(image->size[0]);
  long numCols = (long)(image->size[1]);

  long numRadius = (long)(result->size[0]);
  long numAngles = (long)(result->size[1]);

  real maxRadius = sqrt(numRows * numRows + numCols * numCols) * 0.5;

  real incAngles = 2 * M_PI / numAngles;
  real incRadius = maxRadius / (numRadius - 1);

  real maxImageVal = THTensor_(maxall)(image);

  long row, col, nA ;
  for (row = 0; row < numRows; row ++)
    {
      for (col = 0; col < numCols; col ++)
        {
          //val = img->row, col

          if (*img > 0.05 * maxImageVal)
            {
              //increment vote
              for (nA = 0; nA < numAngles; nA ++)
                {
                  real angle = nA * incAngles;
                  real radiu = sin(angle) * (row-numRows/2) + cos(angle) * (col-numCols/2);
                  real nRR = radiu/incRadius;
                  long nR = (long)(nRR);

                  if (nRR >=0 && nRR < numRadius)
                    {
                      real *tmpHoughPt = res + (nR * numAngles) + nA;
                      *tmpHoughPt += *img;

                    }
                }
            }

          img++;
        }
    }

  return 1;
}

static int libhough_(Main_hough_local_contrast_normalization)(lua_State *L){

  THTensor *result = luaT_checkudata(L, 1, torch_Tensor);
  THTensor *clone = luaT_checkudata(L, 2, torch_Tensor);

  real *res = THTensor_(data)(result);
  real *cln = THTensor_(data)(clone);

  long numRadius = (long)(result->size[0]);
  long numAngles = (long)(result->size[1]);

  int filtsize = 5;

  real *tmpHTNPt = res;
  real *tmpHTOPt = cln;

  long nR, nA, i, j;
  for (nR = 0; nR < numRadius; nR ++){
    for (nA = 0; nA < numAngles; nA ++){
      //tmpHTNPt = res + (nR * numAngles) + nA;
      //tmpHTOPt = cln + (nR * numAngles) + nA;

      real avg;

      // fix weirdness at 45 degree increments
      if (numAngles % 8 == 0 && nA % (long)(numAngles/8) == 0){

        avg = 0.0;
        real cnt = 0.0;

        real *tmpPt;

        for (i = nR-1; i < nR+1; i++){
          for (j = nA-1; j < nA+1; j++){
            long ir = fmin(fmax(0, i), numRadius-1);
            long jc = j % numAngles;
            tmpPt = cln + (ir * numAngles) + jc;
            avg += *tmpPt;
            cnt ++;
          }
        }

        avg = avg/cnt;

        if (*tmpHTOPt > avg){
          *tmpHTNPt = avg;
          *tmpHTOPt = avg;
        }

      }

      // normalization
      if(*tmpHTNPt > 0){

        avg = 0.0;
        real cnt = 0.0;

        real *tmpPt;

        for (i = nR-filtsize; i < nR+filtsize; i++){
          for (j = nA-filtsize; j < nA+filtsize; j++){
            long ir = fmin(fmax(0, i), numRadius-1);
            long jc = j % numAngles;
            tmpPt = cln + (ir * numAngles) + jc;
            avg += *tmpPt;
            cnt ++;
          }
        }

        avg = avg/cnt;

        if (*tmpHTNPt <= avg) {
          *tmpHTNPt = 0;
        }
        else {
          *tmpHTNPt = *tmpHTNPt - avg;
        }
      }

      tmpHTNPt ++;
      tmpHTOPt ++;

    }
  }

  tmpHTNPt = res;
  tmpHTOPt = cln;

  return 1;
}

/*
  ============================================================
  Register functions in LUA

*/

static const struct luaL_reg libhough_(Main__) [] =
{
  {"houghTransform",              libhough_(Main_houghTransform)},
  {"localContrastNormalization",  libhough_(Main_hough_local_contrast_normalization)},
  {NULL,NULL}
};

static void libhough_(Main_init) (lua_State *L)
{
  //luaT_pushmetatable(L, torch_Tensor);
  luaT_registeratname(L, libhough_(Main__), "libhough");
  lua_pop(L,1);
}

#endif
