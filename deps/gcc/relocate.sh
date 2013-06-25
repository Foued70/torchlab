#!/bin/bash

src_dir=$1
install_root=$2

for src_file in `find ${src_dir} -type f`
do 
    dst_file=${install_root}`echo ${src_file} | sed 's!'${src_dir}'!!'`
    dst_dir=`dirname ${dst_file}`

    mkdir -p ${dst_dir}
    mv -f ${src_file} ${dst_dir}
done