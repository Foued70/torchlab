#!/usr/bin/env bash

install_dir=$1

declare -a executables=(fakenect tiltdemo regview record glview glpclview cppview hiview)

for ex in ${executables[@]} 
do 
    for lib in `otool -L ${install_dir}/bin/${ex} | grep '^[^/]*lib' | awk '{ print $1 }'` 
    do
        echo install_name_tool -change ${lib} ${install_dir}/lib/${lib} ${install_dir}/bin/${ex}
        install_name_tool -change ${lib} ${install_dir}/lib/${lib} ${install_dir}/bin/${ex}
    done
done