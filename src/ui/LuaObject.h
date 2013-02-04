#ifndef _LUA_OBJECT_H
#define _LUA_OBJECT_H

#include <sstream>
#include <unordered_map>
#include <vector>
#include <string>
#include <typeinfo>

extern "C" {
#include <TH.h>
#include <luaT.h>
}

typedef std::vector<const std::type_info*> TypeList;

class LuaList;
class LuaObject;

class LuaContainer {
public:
  ~LuaContainer();

protected:
  std::vector<void*> toBeFreed;
  TypeList toBeFreedType;

  void* getValue(lua_State *L, int index);
  void* extractObjectOrList(lua_State *L, int index);
  LuaObject* extractObject(lua_State *L, int index);
  LuaList* extractList(lua_State *L, int index);

};

class LuaList : public std::vector<void*>, public LuaContainer {

public:
  LuaList(lua_State *L, int index);
};



class LuaObject : public std::unordered_map<std::string, void*>, public LuaContainer {

public:
  LuaObject(lua_State *L, int index);
  LuaObject* getObject(const char* name);
  lua_Number* getNumber(const char* name);
  char* getString(const char* name);
};

#endif  /* _LUA_OBJECT_H */
