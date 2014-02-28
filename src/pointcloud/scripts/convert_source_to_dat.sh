#!/bin/bash

SRCDIR=$1

for sweep in $(ls $SRCDIR)
do
  TMPSRC=$SRCDIR"/"$sweep
  cloudlab convert_source_to_dat.lua -srcdir $TMPSRC
done
