basename=chp_
for i in `seq $1 3 $2` 
do 
one=`ls ${basename}*tiff | grep ${basename}'00*'${i}'.tiff'`
two=`ls ${basename}*tiff | grep ${basename}'00*'$((i + 1))'.tiff'` 
thr=`ls ${basename}*tiff | grep ${basename}'00*'$((i + 2))'.tiff'`
outname=${one%%.tiff}_${two%%.tiff}_${thr%%.tiff}.tiff
echo combining ${one} ${two} ${thr} $outname 
# align_image_stack creates an HDR in radiance .hdr format only.
# align_image_stack -v -o ${outname} ${one} ${two} ${thr}

# enfuse is for creating a LDR image which looks perceptually good.
# More pleasing than tone mapping in many respects
enfuse -d 8 -o ${outname} ${one} ${two} ${thr}
done
