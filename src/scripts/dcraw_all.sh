#!/bin/bash
# for dcraw processing of all raw files in source folder

PRJDIR=$1 # /Users/lihui815/Documents/elegant-prize-3149
SRCDIR=$PRJDIR"/source/po_scan/a"
WRKDIR=$PRJDIR"/work/a_00/Images"

mkdir $PRJDIR"/work"
mkdir $PRJDIR"/work/a_00"
mkdir $WRKDIR

for sweep in $(ls $SRCDIR)
do
  sweepsrc=$SRCDIR"/"$sweep
  sweeptgt=$WRKDIR"/"$sweep
  
  echo $sweepsrc
  echo $sweeptgt
  
  mkdir $sweeptgt
  
  sh dcraw.sh $sweepsrc"/*.nef"
  
  for imgp in $(ls $sweepsrc"/"*.tiff)
  do
    img=$(basename $imgp)
    mv $sweepsrc"/"$img $sweeptgt"/"$img
  done
  
done