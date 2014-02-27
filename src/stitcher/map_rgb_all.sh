#!/bin/bash
# for aligning all images in a folder

PRJDIR=$1 # /Users/lihui815/Documents/elegant-prize-3149
WRKDIR=$PRJDIR"/work/a_00/Aligned_For_Use"

for sweep in $(ls $WRKDIR)
do
  
  echo $sweep
  $(cloudlab map_rgb.lua -prjdir $PRJDIR -swpdir $sweep)
  
done