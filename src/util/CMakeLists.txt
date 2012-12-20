CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
FIND_PACKAGE(Torch REQUIRED)

FILE(GLOB luasrc [a-zA-Z]*.lua)
FILE(GLOB testlua test/[a-zA-Z]*.lua)
SET(luasrc ${luasrc} ${testlua})

ADD_TORCH_PACKAGE(util "" "${luasrc}" "Floored 3D utils")
