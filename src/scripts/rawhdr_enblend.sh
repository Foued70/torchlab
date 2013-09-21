#!/bin/bash
dcraw=${CLOUDLAB_INSTALL_ROOT}/bin/dcraw.sh
i=1
fls=
outname=
for f in $*
do
    echo $f ;
    ${dcraw} $f ;
    fls="$fls ${f%%nef}tiff"
    if [ $outname ]
    then
        outname="$outname-`basename ${f%%.nef}`"
    else
        outname=`basename ${f%%.nef}`
    fi
    if [ $((i % 3)) == 0 ]
    then
        enfuse -d 8 -o ${outname}.tiff $fls
        rm $fls
        fls=
        outname=
    fi
    i=$((i+1))
done
