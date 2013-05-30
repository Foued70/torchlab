# rough script to downsize images
# preserve exif information by copying first and then using mogrify
indir=$1
itype=.JPG
size=1024
outdir=${1%%/}_${size}
mkdir -p $outdir
for i in ${indir}/*${itype} 
do 
    bn=`basename $i`
    o=${outdir}/${bn}
    echo $i '->' $o
    cp $i $o
    mogrify -resize ${size} $o
done