-- This file contains two parts.  First the ffi interface to opencv
-- and our C wrappers for opencv. Then the lua wrappers.

-- The lua wrappers to C functions which call the opencv C++
-- functions. The ffi interface is in libopencv.lua and the c wrapper
-- functions are written in luaopencv.cpp.

-- All functions in the opencv package take pointers to opencv objects
-- as input.  There are functions to convert to and from torch.Tensors
-- which don't copy data when then can.

-- TODO figure out how to put this in another file. To make code more readable.

-- FFI bindings to Opencv:
local ffi = require "ffi"
-- local bit = require 'bit'
local ctorch = util.ctorch

-- data types CV_<bit_depth>{U|S|F}C{num_channels}

-- copy needed structs from opencv2/core/types_c.h
-- these will create the interface between torch and opencv

ffi.cdef
[[

typedef unsigned char uchar;

typedef struct FeatureExtractor FeatureExtractor;

// ------------
//   transparent pointers (internals accessible from Lua)
// ------------
typedef struct CvPoint
{
    int x;
    int y;
}
CvPoint;


typedef struct CvPoint3D32f
{
    float x;
    float y;
    float z;
}
CvPoint3D32f;

typedef struct CvRect
{
    int x;
    int y;
    int width;
    int height;
} CvRect;

typedef struct CvSize
{
  int width;
  int height;
} CvSize;


]]


-- load our C wrappers. TODO expose C or not...
libopencv = util.ffi.load('libcopencv')

return libopencv
