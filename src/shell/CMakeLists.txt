CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
include(../common.cmake REQUIRED)

find_path(Readline_ROOT_DIR
    NAMES include/readline/readline.h
)

find_library(Readline_LIBRARY
    NAMES readline
    HINTS ${Readline_ROOT_DIR}/lib
)

ADD_LUVIT_LIB(readline_async "readline_async.c")

TARGET_LINK_LIBRARIES(readline_async ${Readline_LIBRARY})
