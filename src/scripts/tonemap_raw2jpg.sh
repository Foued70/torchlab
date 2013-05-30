
PROJ_DIR=$PWD

if [ "$#" -gt 0 ]
then
    PROJ_DIR="$1"
fi

if [ ! -d $PROJ_DIR ]
then
    echo no such project directory: $PROJ_DIR
    exit 1;
fi

echo $PROJ_DIR

CONTRAST=50.0

CCMD=tone_mapping
for sweepnum in $(ls $PROJ_DIR)
do
    if [ -d "$PROJ_DIR"/"$sweepnum" ]
    then
        SWEEP_DIR="$PROJ_DIR"/"$sweepnum"
        PPM_DIR="$SWEEP_DIR"/PPM
        JPG_DIR="$SWEEP_DIR"/JPG
        EXR_DIR="$SWEEP_DIR"/EXR
        NEF_DIR="$SWEEP_DIR"/NEF

        mkdir $PPM_DIR
        mkdir $JPG_DIR
        mkdir $EXR_DIR
        mkdir $NEF_DIR

        mv "$SWEEP_DIR"/*.NEF "$NEF_DIR"/

        for nefname in $(ls "$NEF_DIR")
        do
            echo ""
            echo "$SWEEP_DIR" / "$nefname"
            echo ""

            namelen=${#nefname}
            fname=${nefname:0:$namelen-4}
            ppmname_i="$PPM_DIR"/"$fname"_i.ppm
            exrname="$EXR_DIR"/"$fname".exr
            ppmname_o="$PPM_DIR"/"$fname"_o.ppm
            nefname="$NEF_DIR"/"$nefname"

            dcraw -c -6 -W "$nefname" > "$ppmname_i"
            ppmtoexr "$ppmname_i" "$exrname"
            CCMD=$(echo $CCMD "$exrname" "$ppmname_o")
        done
    fi
done

CCMD=$(echo $CCMD $CONTRAST)
$CCMD

for sweepnum in $(ls $PROJ_DIR)
do
    if [ -d "$PROJ_DIR"/"$sweepnum" ]
    then
        SWEEP_DIR="$PROJ_DIR"/"$sweepnum"

        PPM_DIR="$SWEEP_DIR"/PPM
        JPG_DIR="$SWEEP_DIR"/JPG
        EXR_DIR="$SWEEP_DIR"/EXR
        NEF_DIR="$SWEEP_DIR"/NEF

        for ppmname in $(ls "$PPM_DIR"/*_o.ppm)
        do
            namelen=${#ppmname}
            fname=${ppmname:($namelen-14):$namelen}
            namelen=${#fname}
            fname=${fname:0:$namelen-6}
            jpgname="$JPG_DIR"/$fname.jpg
            echo $ppmname $jpgname
            cjpeg -quality 100 "$ppmname" > "$jpgname"
        done
    fi
done