#!/usr/bin/env bash

install_dir=$1

lib_dir=${install_dir}/lib/

for boost_dylib in ${lib_dir}/libboost_*dylib
do 
    otool -D $boost_dylib
    install_name_tool -id $boost_dylib $boost_dylib
    for lib in `otool -L $boost_dylib | grep 'libboost' | awk '{ print $1 }'`
    do
        echo $lib
        install_name_tool -change $lib ${lib_dir}/`basename ${lib}` ${boost_dylib}
    done
done
