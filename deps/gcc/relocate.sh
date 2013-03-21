#!/bin/bash

TAR_DIR=$1
INSTALL_ROOT=$2

for SRC_FILE in `find ${TAR_DIR} -type f`
do 
    DST_FILE=${INSTALL_ROOT}`echo ${SRC_FILE} | sed 's!'${TAR_DIR}'!!'`
    DST_DIR=`dirname ${DST_FILE}`

    mkdir -p ${DST_DIR}
    mv -f ${SRC_FILE} ${DST_DIR}
done