#!/bin/bash
# for stitching all sweeps in a folder

PRJDIR=$1 # /Users/lihui815/Documents/precise-transit-6548
SRCDIR=$PRJDIR"/work/a_00/Images"

for sweep in $(ls $SRCDIR)
do
  
  $(cloudlab stitch_panorama.lua -srcdir $PRJDIR -swdir $sweep)
  
done