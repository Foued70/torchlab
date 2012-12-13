
CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
FIND_PACKAGE(Torch REQUIRED)

FILE(GLOB luasrc *.lua)
SET(luasrc ${luasrc} test/geom-test.lua)
SET(luasrc ${luasrc} test/geom-data.lua)

ADD_TORCH_PACKAGE(utils "" "${luasrc}" "Floored 3D utils")
