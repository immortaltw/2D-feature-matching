# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Report Generation Instructions
`./report_gen.sh`

Then check [final_report.csv](./final_report.csv)

## Result

The result of all detector / descriptor combination is shown in the table below. As the result sugguests, the top 3 fastest detector / descriptor combinations are:

1. FAST-ORB
2. ORB-BRIEF
3. FAST-BRIEF

The result shows binary-based algorithms provides very good result in turns of computation speed. As for the correctness metric, we would need to have a groud-truth data to generate the ROC diagrams to get a better understanding. Notice that in this project we didn't fine-tune the parameters of each algorithm pairs. The result might be different if we plugged in different parameters.

| combinations           | avg detectTime        | avg computeTime       | feature extraction time | 
|------------------------|-----------------------|-----------------------|-------------------------| 
| FAST-ORB-MAT_BF        | 0.0017186570000000002 | 0.005073871           | 0.006792528             | 
| ORB-BRIEF-MAT_BF       | 0.007934156           | 0.0013939134000000002 | 0.0093280694            | 
| FAST-BRIEF-MAT_BF      | 0.001652352           | 0.008186039           | 0.009838391             | 
| SHITOMASI-BRIEF-MAT_BF | 0.008351275           | 0.0025516249999999996 | 0.0109029               | 
| SHITOMASI-ORB-MAT_BF   | 0.009532331           | 0.0032524529999999998 | 0.012784784             | 
| HARRIS-BRIEF-MAT_BF    | 0.011912451           | 0.0013847717000000002 | 0.0132972227            | 
| HARRIS-ORB-MAT_BF      | 0.012135683999999999  | 0.002785233           | 0.014920916999999999    | 
| ORB-ORB-MAT_BF         | 0.0074799440000000005 | 0.008754712000000001  | 0.016234656             | 
| ORB-FREAK-MAT_BF       | 0.007742710999999999  | 0.020134080000000002  | 0.027876791             | 
| SHITOMASI-FREAK-MAT_BF | 0.008556600000000001  | 0.023505279999999996  | 0.03206188              | 
| FAST-FREAK-MAT_BF      | 0.001832862           | 0.030659469999999994  | 0.03249233199999999     | 
| HARRIS-FREAK-MAT_BF    | 0.012337319999999999  | 0.020689680000000002  | 0.033027                | 
| SIFT-BRIEF-MAT_BF      | 0.07083978            | 0.0033836370000000005 | 0.074223417             | 
| AKAZE-AKAZE-MAT_BF     | 0.052216649999999996  | 0.04418821            | 0.09640486              | 
| SIFT-FREAK-MAT_BF      | 0.07010042000000001   | 0.026307919999999995  | 0.09640834000000001     | 
| SIFT-SIFT-MAT_BF       | 0.07044189            | 0.053618299999999994  | 0.12406018999999999     | 
| ORB-BRISK-MAT_BF       | 0.008951978           | 0.12484899999999999   | 0.133800978             | 
| SHITOMASI-BRISK-MAT_BF | 0.009024782           | 0.127747              | 0.136771782             | 
| HARRIS-BRISK-MAT_BF    | 0.013974951999999999  | 0.12613570000000002   | 0.140110652             | 
| FAST-BRISK-MAT_BF      | 0.001675265           | 0.14309260000000004   | 0.14476786500000005     | 
| BRISK-BRIEF-MAT_BF     | 0.15654479999999998   | 0.005793742           | 0.16233854199999997     | 
| BRISK-ORB-MAT_BF       | 0.1576519             | 0.010168874999999999  | 0.167820775             | 
| BRISK-FREAK-MAT_BF     | 0.157144              | 0.02944704            | 0.18659104              | 
| SIFT-BRISK-MAT_BF      | 0.07291247000000001   | 0.14358949999999998   | 0.21650197              | 
| BRISK-BRISK-MAT_BF     | 0.1564187             | 0.13402920000000001   | 0.2904479               | 
