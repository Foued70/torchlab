#!/bin/bash

install_dir=`pwd`

# relocate all libs from hpc's gcc
for f in *dylib
do
  echo install_name_tool -id ${install_dir}/$f $f
  install_name_tool -id ${install_dir}/$f $f
  for l in `otool -L $f | grep '\t/usr/local/lib' | awk '{print $1}' `
  do
    nl=`echo $l | sed 's!/usr/local/lib!'$install_dir'!'`
    echo install_name_tool -change $l $nl $f
    install_name_tool -change $l $nl $f
  done
done