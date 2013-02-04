#include "LuaObject.h"
#include "utils.h"

const std::type_info* LuaListType = &typeid(LuaList);
const std::type_info* LuaObjectType = &typeid(LuaObject);
const std::type_info* LuaNumberType = &typeid(lua_Number);


LuaObject::LuaObject(lua_State *L, int index) {
  lua_pushnil(L);
  while (lua_next(L, index) != 0) {
    /* uses 'key' (at index -2) and 'value' (at index -1) */
    const char* name = luaL_checkstring(L, -2);
    // log(PARAM, "  %s", name);
    (*this)[name] = getValue(L, -1);

    /* removes 'value'; keeps 'key' for next iteration */
    lua_pop(L, 1);
  }
}

LuaObject* 
LuaObject::getObject(const char* name) {
  return (LuaObject*)((*this)[name]);
}

lua_Number* 
LuaObject::getNumber(const char* name) {
  return (lua_Number*)((*this)[name]);
}

char* 
LuaObject::getString(const char* name) {
  return (char*)((*this)[name]);
}


LuaList::LuaList(lua_State *L, int index) {
  lua_pushnil(L);
  while (lua_next(L, index) != 0) {
    /* uses 'key' (at index -2) and 'value' (at index -1) */
    lua_Number j = luaL_checknumber(L, -2);
    // convert to int handling float rounding issue
    int k = (int)(j+(j<0?-0.5:0.5));

    if (this->size() < k) {
      this->resize(k, NULL);
    }

    (*this)[k-1] = getValue(L, -1);

    // log(PARAM, "  %d %d", k, list->size());
    
    /* removes 'value'; keeps 'key' for next iteration */
    lua_pop(L, 1);
  }
}


void*
LuaContainer::extractObjectOrList(lua_State *L, int index) {
  // log(PARAM, "extractObjectOrList(L, %d)", index);

  if (index < 0) index = lua_gettop(L) + index + 1;

  lua_pushnil(L);
  if (lua_next(L, index) != 0) {
    if (lua_isnumber(L, -2)) {
      lua_pop(L, 2);
      return extractList(L, index);
    }
    else {
      lua_pop(L, 2);
      return extractObject(L, index);
    }
  }
  else {
    return extractList(L, index);
  }
}


LuaObject*
LuaContainer::extractObject(lua_State *L, int index) {
  // log(PARAM, "extractObject(L, %d)", index);

  if (index < 0) index = lua_gettop(L) + index + 1;

  LuaObject* obj = new LuaObject(L, index);
  
  toBeFreed.push_back(obj);
  toBeFreedType.push_back(LuaObjectType);

  return obj;
}


LuaList*
LuaContainer::extractList(lua_State *L, int index) {
  // log(PARAM, "extractList(L, %d)", index);

  if (index < 0) index = lua_gettop(L) + index + 1;

  LuaList* list = new LuaList(L, index);

  toBeFreed.push_back(list);
  toBeFreedType.push_back(LuaListType);

  return list;
}

void*
LuaContainer::getValue(lua_State *L, int index) {
  // log(PARAM, "getValue(L, %d)", index);
  void* value = NULL;
  if (luaT_isudata(L, index, "torch.DoubleTensor")) {
    value = (void*)luaT_toudata(L, index, "torch.DoubleTensor");
  }
  else if (luaT_isudata(L, index, "torch.IntTensor")) {
    value = (void*)luaT_toudata(L, index, "torch.IntTensor");
  }
  else if (lua_isnumber(L, index)) {
    lua_Number* val = new lua_Number;
    *val = luaL_checknumber(L, index);

    value = val;

    toBeFreed.push_back(value);
    toBeFreedType.push_back(LuaNumberType);
  }
  else if (lua_isstring(L, index)) {
    value = (void*)luaL_checkstring(L, index);
  }
  else if (lua_istable(L, index)) {
    value = (void*)extractObjectOrList(L, index);
  }
  else {
      log(WARN, "LuaObject: unknown value type (%s)", lua_typename(L, lua_type(L, index)));
  }

  return value;
}


LuaContainer::~LuaContainer() {
  // log(PARAM, "~LuaContainer");
  for (int i=0; i<toBeFreed.size(); i++) {
    if (toBeFreedType[i] == LuaListType) {
      // log(PARAM, "~LuaList");
      delete (LuaList*)toBeFreed[i];
    }
    else if (toBeFreedType[i] == LuaObjectType) {
      // log(PARAM, "~LuaObject");
      delete (LuaObject*)toBeFreed[i];
    }
    else if (toBeFreedType[i] == LuaNumberType) {
      // log(PARAM, "~lua_Number");
      delete (lua_Number*)toBeFreed[i];
    }
  }

}

