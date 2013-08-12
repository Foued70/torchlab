#include <stdio.h>
#include <stdlib.h>
#include "readline/readline.h"
#include "readline/history.h"
#include "luvit/luv.h"
#include "uv.h"
#include "lua.h"
#include "lauxlib.h"

const char* CLOUDLAB_HISTORY_FILE_SUFFIX = "/.cloudlab_history";
char* CLOUDLAB_HISTORY_FILE = NULL;

uv_thread_t readline_thread;
uv_mutex_t readline_mutex;
uv_async_t* readline_async;
uv_sem_t readline_sem;

typedef struct {
  const char *word;
  char* line;
  int startpos;
  int endpos;
  char** completions;
} readline_data_t;

readline_data_t readline_data;

char* readline_prompt = NULL;

typedef enum {COMPLETE, EXECUTE, CLOSE} action_t;
action_t action;

void async_send_and_wait(action_t act) {
  action = act;
  uv_async_send(readline_async);
  uv_mutex_unlock(&readline_mutex);
  uv_sem_wait(&readline_sem);
}

char* completion_entry(const char *word, int state) {
  if (readline_data.completions) {
    return readline_data.completions[state];
  }

  return NULL;
}

char** completion(const char *word, int startpos, int endpos)
{
  uv_mutex_lock(&readline_mutex);

  readline_data.word = word;
  readline_data.startpos = startpos;
  readline_data.endpos = endpos;

  async_send_and_wait(COMPLETE);

  if (readline_data.completions && readline_data.completions[0]) {
    return rl_completion_matches(word, completion_entry);
  }
  else {
    return NULL;
  }
}

void readline_loop(void *arg) {
  char* line = (char *)NULL;
  
  while (TRUE) {
    if (line) {
      free(line);
      line = (char *)NULL;
    }

    char* line = readline(readline_prompt);
    if (line) {
      uv_mutex_lock(&readline_mutex);
      readline_data.line = line;
      async_send_and_wait(EXECUTE);
    }
    else {
      printf("\n");
      uv_mutex_lock(&readline_mutex);
      async_send_and_wait(CLOSE);
    }
  }

}


void do_action(uv_async_t *handle, int status) {
  uv_mutex_lock(&readline_mutex);

  lua_State *L = luv_handle_get_lua(handle->data);

  if (action == COMPLETE) {
    // printf("\npost: %s\n", readline_data.word);
    lua_pushstring(L, readline_data.word);
    lua_pushstring(L, rl_line_buffer);
    lua_pushinteger(L, readline_data.startpos);
    lua_pushinteger(L, readline_data.endpos);
    luv_emit_event(L, "complete", 4);
    // printf("event:\n");
  }
  else if (action == EXECUTE) {
    lua_pushstring(L, readline_data.line);
    luv_emit_event(L, "execute", 1);
  }
  else if (action == CLOSE) {
    luv_emit_event(L, "close", 0);
  }

  uv_sem_post(&readline_sem);
  uv_mutex_unlock(&readline_mutex);
}

char* copy_string(char* source) {
  char* dest = malloc(1 + strlen(source));
  strcpy(dest, source);
  return dest;
}

int set_completions(lua_State* L) {
  size_t len = lua_objlen(L, 1);

  readline_data.completions = calloc(len+1, sizeof(char*));
  
  int i;
  for(i = 0; i < len; i++){
    lua_rawgeti(L, 1, i+1);
    char* comp = (char*)luaL_checkstring(L, -1);
    readline_data.completions[i] = copy_string(comp);
    lua_pop(L, 1);
  }
  // printf("set:\n");
  return 0;
}


uv_async_t* fluv_create_async(lua_State* L) {
  return (uv_async_t*)(luv_handle_create(L, sizeof(uv_async_t), "fluv_async")->handle);
}


int start_readline_loop(lua_State* L) {
  char* home = getenv("HOME");
  CLOUDLAB_HISTORY_FILE = malloc(strlen(home) + strlen(CLOUDLAB_HISTORY_FILE_SUFFIX) + 1);
  strcpy(CLOUDLAB_HISTORY_FILE, home);
  strcat(CLOUDLAB_HISTORY_FILE, CLOUDLAB_HISTORY_FILE_SUFFIX);

  FILE* fp = fopen(CLOUDLAB_HISTORY_FILE, "ab+");
  fclose(fp);

  rl_readline_name = "cloudlab";
  rl_basic_word_break_characters = " \t\n\"'><=;:+-*%^~#{}()[].,";
  rl_attempted_completion_function = completion;
  using_history();
  stifle_history(1000);
  int res = read_history(CLOUDLAB_HISTORY_FILE);

  readline_data.completions = (char**)NULL;

  readline_async = fluv_create_async(L);

  uv_async_init(luv_get_loop(L), readline_async, do_action);
  uv_sem_init(&readline_sem, 0);
  uv_mutex_init(&readline_mutex);
  uv_thread_create(&readline_thread, readline_loop, NULL);
  luv_handle_ref(L, readline_async->data, 1);

  return 1;
}

int set_prompt(lua_State* L) {
  char* prompt = (char*)luaL_checkstring(L, 1);
  if (readline_prompt) free(readline_prompt);
  readline_prompt = copy_string(prompt);
  return 0;
}


int add_to_history(lua_State* L) {
  char* command = (char*)luaL_checkstring(L, 1);
  add_history(command);
  int res = write_history(CLOUDLAB_HISTORY_FILE);
  return 0;
}


int set_completion_append_character(lua_State* L) {
  rl_completion_append_character = luaL_checkinteger(L, 1);
  return 0;
}


static const luaL_reg libshell[] = {
  {"start_readline_loop", start_readline_loop},
  {"set_completions", set_completions},
  {"set_prompt", set_prompt},
  {"add_to_history", add_to_history},
  {"set_completion_append_character", set_completion_append_character},
  {NULL, NULL}
};

LUALIB_API int luaopen_libreadline_async(lua_State *L) {

  lua_newtable (L);
  luaL_register(L, NULL, libshell);

  /* Return the new module */
  return 1;
}

