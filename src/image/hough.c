#include <TH.h>
#include <luaT.h>


#define torch_(NAME) TH_CONCAT_3(torch_, Real, NAME)
#define torch_Tensor TH_CONCAT_STRING_3(torch.,Real,Tensor)
#define libhough_(NAME) TH_CONCAT_3(libhough_, Real, NAME)


#include "generic/hough.c"
#include "THGenerateFloatTypes.h"

DLL_EXPORT int luaopen_libhough(lua_State *L)
{

  lua_newtable(L);
  lua_pushvalue(L, -1);
  lua_setfield(L, LUA_GLOBALSINDEX, "libhough");

  libhough_FloatMain_init(L);
  libhough_DoubleMain_init(L);

  return 1;
}
