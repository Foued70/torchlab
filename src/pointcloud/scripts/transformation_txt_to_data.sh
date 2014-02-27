#!/bin/bash

SRCDIR=$1
TRFDIR=$2

for sweep in $(ls $SRCDIR)
do
  cloudlab transformation_txt_to_data.lua -readdir $SRCDIR -writdir $TRFDIR -fname $sweep
done