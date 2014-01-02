#!/bin/bash
# for dcraw processing of all raw files in source folder

PRJDIR=$1 # /Users/lihui815/Documents/elegant-prize-3149

$(sh dcraw_all.sh $PRJDIR)
$(sh align_all_images.sh $PRJDIR)