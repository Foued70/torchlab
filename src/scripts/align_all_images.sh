#!/bin/bash
# for aligning all images in a folder

PRJDIR=$1 # /Users/lihui815/Documents/elegant-prize-3149
SRCDIR=$PRJDIR"/work/a_00/Images"
WRKDIR=$PRJDIR"/work/a_00/Aligned"

mkdir $WRKDIR

for sweep in $(ls $SRCDIR)
do

  sweepsrc=$SRCDIR"/"$sweep
  sweeptgt=$WRKDIR"/"$sweep
  
  mkdir $sweeptgt
  
  $(cloudlab align_images.lua -imagedir $sweepsrc -outimage $sweeptgt"/"panorama_360)
  
done