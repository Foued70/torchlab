# from a directory of images, splits into image types then by exposure
# JPG/s15 /, JPG/s30, ... NEF/s15, NEF/s30 etc.
#
# this is a very simple script which should be extended.  Makes several assumptions.
# 
# + the HDRs where created by bracketing Exposure
# + JPGs and NEF (Nikon raw format) are to be split
#
# Dependencies: exiftool
# 

declare -a image_types=("JPG" "NEF")

for itype in "${image_types[@]}"
do 
    echo "processing $itype"
    mkdir -p $itype
    mv *.$itype $itype
    exps=`exiftool -ExposureTime *${itype} | \
            grep Exposure | sort | uniq | sed 's/.*: //'`
    echo cd $itype
    cd $itype
    for e in $exps
    do 
        dname=`echo $e | sed -e 's!1/!!' -e 's/^/s/'`
        mkdir -p $dname
    done
    for f in *.$itype 
    do 
        dname=`exiftool -ExposureTime $f | \
                 grep Exposure | sed -e 's/.*: //' -e 's!1/!!' -e 's/^/s/'` 
        echo mv $f $dname
        mv $f $dname
    done
    cd -
done	 
