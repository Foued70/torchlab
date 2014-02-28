#!/bin/bash

SRCDIR=$1
TRFDIR=$2

for sweep in $(ls $SRCDIR)
do
  cloudlab load_transformations.lua -pcddir $SRCDIR -trfdir $TRFDIR -swpnum $sweep
done