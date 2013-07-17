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
  real considerVal = 0.01 * maxImageVal;
  real ptExistWght = 0.10 * maxImageVal;

  long row, col, nA ;
  for (row = 0; row < numRows; row ++)
    {
      for (col = 0; col < numCols; col ++)
        {
          //val = img->row, col

          if (*img > considerVal)
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
                    *tmpHoughPt += *img + ptExistWght;

                  }
                }
            }

          img++;
        }
    }

  return 1;
}

static int libhough_(Main_houghTransformVertical)(lua_State *L){
  THTensor *result = luaT_checkudata(L, 1, torch_Tensor);
  THTensor *image  = luaT_checkudata(L, 2, torch_Tensor);

  real *res = THTensor_(data)(result);
  real *img = THTensor_(data)(image);

  long numRows = (long)(image->size[0]);
  long numCols = (long)(image->size[1]);

  long numRadius = (long)(result->size[0]);
  long numAngles = (long)(result->size[1]);

  real maxRadius = numCols;

  real incAngles = 2 * M_PI / numAngles;
  real incRadius = maxRadius / (numRadius - 1);

  real maxImageVal = THTensor_(maxall)(image);
  real considerVal = 0.01 * maxImageVal;
  real ptExistWght = 0.10 * maxImageVal;

  long row, col, nA ;
  for (row = 0; row < numRows; row ++)
    {
      for (col = 0; col < numCols; col ++)
        {
          //val = img->row, col

          if (*img > considerVal)
            {
              //increment vote
              for (nA = 0; nA < numAngles; nA ++)
                {
                  //real angle = nA * incAngles;
                  real angle = 0;
                  real radiu = col;
                  real nRR = radiu/incRadius;
                  long nR = (long)(nRR);

                  if (nRR >=0 && nRR < numRadius)
                  {
                    real *tmpHoughPt = res + (nR * numAngles) + nA;
                    *tmpHoughPt += *img + ptExistWght;

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

  int filtsize = (numRadius+numAngles)/500 + 1;

  real *tmpHTNPt = res;
  real *tmpHTOPt = cln;

  long nR, nA, i, j;
  for (nR = 0; nR < numRadius; nR ++){
    for (nA = 0; nA < numAngles; nA ++){

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
  {"houghTransformVertical",       libhough_(Main_houghTransformVertical)},
  {"localContrastNormalization",  libhough_(Main_hough_local_contrast_normalization)},
  {NULL,NULL}
};

static void libhough_(Main_init) (lua_State *L)
{
  luaT_pushmetatable(L, torch_Tensor);
  luaT_registeratname(L, libhough_(Main__), "libhough");
  lua_pop(L,1);
}

#endif
