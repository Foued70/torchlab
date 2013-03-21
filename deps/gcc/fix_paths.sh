#!/bin/bash

INSTALL_DIR=`pwd`

# relocate all libs from hpc's gcc
for f in *dylib
do
  echo install_name_tool -id ${INSTALL_DIR}/$f $f
  install_name_tool -id ${INSTALL_DIR}/$f $f
  for l in `otool -L $f | grep '\t/usr/local/lib' | awk '{print $1}' `
  do
    nl=`echo $l | sed 's!/usr/local/lib!'$INSTALL_DIR'!'`
    echo install_name_tool -change $l $nl $f
    install_name_tool -change $l $nl $f
  done
done