basename=chp_
for i in `seq -w $1 3 $2` 
do 
one=`ls ${basename}*tiff | grep ${basename}'0*'${i}.tiff`
two=`ls ${basename}*tiff | grep ${basename}'0*'$((i + 1)).tiff` 
thr=`ls ${basename}*tiff | grep ${basename}'0*'$((i + 2)).tiff`
outname=${one%%.tiff}_${two%%.tiff}_${thr%%.tiff}.hdr
echo $outname 
align_image_stack -v -o ${outname} ${one} ${two} ${thr}
done
