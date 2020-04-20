#!/bin/bash

# Reset
rm -rf report *csv

# Build
rm -rf build && mkdir build && cd build && cmake ../ && make
cd build

# Hyper params
DETECTOR_TYPES="HARRIS SHITOMASI FAST BRISK ORB"
DESCRIPTOR_TYPES="BRISK BRIEF ORB FREAK"
MATCHER_TYPE="MAT_BF"   # MAT_BF MAT_FLANN
DESCRIPTOR_CAT="DES_BINARY" # DES_BINARY DES_HOG
SELECTOR_TYPE="SEL_KNN"  # SEL_NN SEL_KNN

for DET in $DETECTOR_TYPES
do
    for DEC in $DESCRIPTOR_TYPES
    do
        ./2D_feature_tracking $DET $DEC $MATCHER_TYPE DES_BINARY $SELECTOR_TYPE
    done
done

# SIFT
# SIFT (L2_NORM)
DESCRIPTOR_TYPES="BRISK BRIEF FREAK SIFT"
for DEC in $DESCRIPTOR_TYPES
do
    ./2D_feature_tracking SIFT $DEC $MATCHER_TYPE DES_HOG $SELECTOR_TYPE
done

# AKAZE
# AKAZE
./2D_feature_tracking AKAZE AKAZE $MATCHER_TYPE DES_BINARY $SELECTOR_TYPE

cd ../
mkdir report
mv *.csv report/

python3 report_parser.py