# for dcraw processing if there is chromatic aberration (super common
# at extremes in wide angle lenses).  Light of different colors is
# bent different angles throught the light path.  You can test with dcraw.

# use fast interpolation -q 0 and no median filter to best see the abherration.

chromaR=0.9992 # chromatic aberration Red Channel
q=3      # ADH interpolation
m=3      # median filter 3 pass

manual="1.445312 1.000000 1.75 1.000000"
camera_fixed="1.671875 1.000000 1.542969 1.000000"

D65="2.395443 1.000000 1.253807 1"
Tungsten="1.392498 1.000000 2.375114 1"
Daylight="2.132483 1.000000 1.480864 1"
Fluorescent="1.783446 1.000000 1.997113 1"
Shade="2.531894 1.000000 1.223749 1"
Flash="2.429833 1.000000 1.284593 1"
Cloudy="2.336605 1.000000 1.334642 1"

for f in $*
do 
# when creating HDR image we want linear curve
# dcraw -v -q ${q} -m ${m} -C ${chromaR} 1 -4 -T -r ${manual} -o 2 ${f}
# when using enfuse we want to keep camera white and gamma
# -v verbose
# -T tiff output (rather than ppm)
# -6 16 bit output or we are losing dynamic range (advantage of raw)
# -W don't automatically brighten (or we lose all sense of radiance)
# -w use camera white balance (this is for Louvre, should look at using set parameters above)
    dcraw -v -T -6 -W -r ${camera_fixed} ${f}
    
done
