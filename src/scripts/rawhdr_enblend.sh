i=1
fls=
outname=
for f in $* 
do
echo $f ; 
~/proj/cloudlab/src/scripts/dcraw.sh $f ; 
fls="$fls ${f%%NEF}tiff"
outname="$outname`basename ${f%%.NEF}`"
if [ $((i % 3)) == 0 ]
then
enfuse -d 8 -o ${outname}.tiff $fls
rm $fls
fls=
outname=
fi
i=$((i+1))
done
