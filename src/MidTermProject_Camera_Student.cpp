/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // default hyper parameters
    string detectorType = "HARRIS";       // HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType = "ORB";        // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorCat = "DES_BINARY";  // DES_BINARY, DES_HOG
    string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

    // parse args into hyper parameters
    // usage:
    //  ./2D_feature_tracking detectorType descriptorType \
    //     matcherType descriptorCat selectorType
    if (argc == 6) {
        detectorType = string(argv[1]);
        descriptorType = string(argv[2]);
        matcherType = string(argv[3]);
        descriptorCat = string(argv[4]);
        selectorType = string(argv[5]);
    }

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time

    // result writer
    string resultPath = dataPath;
    string resultPrefix = detectorType + "-" + descriptorType + "-" + matcherType;
    string resultType = ".csv";
    ofstream resultCsv(resultPath + resultPrefix + resultType);
    stringstream resultRow;
    stringstream prevResultRow;

    resultCsv << "detectTime";
    resultCsv << ",";
    resultCsv << "keypoints";
    resultCsv << ",";
    resultCsv << "keypointsDist";
    resultCsv << ",";
    resultCsv << "computeTime";
    resultCsv << ",";
    resultCsv << "matched";
    resultCsv << '\n';

    //// TASK MP.1 Initialized here...
    DataBuffer<DataFrame> dataBuffer(dataBufferSize); // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        // perform feature description
        double t1 = (double)cv::getTickCount();
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }

        t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
        resultRow << t1 << ',';
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            for (int i=0; i<keypoints.size(); ++i)
            {
                auto kp = keypoints[i].pt;
                if (vehicleRect.x > kp.x ||
                    vehicleRect.y > kp.y ||
                    vehicleRect.x + vehicleRect.width < kp.x ||
                    vehicleRect.y + vehicleRect.height < kp.y)
                {
                    keypoints.erase(keypoints.begin()+i);
                }
            }
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.at(-1).keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;
        resultRow << keypoints.size() << ',';

        map<int, int> kpNeighborhoodSizeDist;
        for (auto kp : keypoints) {
            kpNeighborhoodSizeDist[kp.size]++;
        }

        cout << "Neighborhood Size Dist" << endl;
        stringstream ssKpNeighbor;
        ssKpNeighbor << "\"{";
        int kpCnt = 0;
        for (auto k : kpNeighborhoodSizeDist) {
            ssKpNeighbor << k.first << ":" << k.second << (kpCnt < kpNeighborhoodSizeDist.size()-1? ",": "");
            kpCnt++;
        }
        ssKpNeighbor << "}\"";
        cout << ssKpNeighbor.str() << endl;
        resultRow << ssKpNeighbor.str() << ',';
        ssKpNeighbor.str({});

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;

        double t2 = (double)cv::getTickCount();
        descKeypoints(dataBuffer.at(-1).keypoints, dataBuffer.at(-1).cameraImg, descriptors, detectorType, descriptorType);
        t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
        resultRow << t2 << ',';
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        dataBuffer.at(-1).descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */
            vector<cv::DMatch> matches;

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            matchDescriptors(dataBuffer.at(-2).keypoints, dataBuffer.at(-1).keypoints,
                             dataBuffer.at(-2).descriptors, dataBuffer.at(-1).descriptors,
                             matches, descriptorCat, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            dataBuffer.at(-1).kptMatches = matches; 

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            cout << "Num of matched keypoints: " << matches.size() << endl;
            prevResultRow << matches.size();

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = (dataBuffer.at(-1).cameraImg).clone();
                cv::drawMatches(dataBuffer.at(-2).cameraImg, dataBuffer.at(-2).keypoints,
                                dataBuffer.at(-1).cameraImg, dataBuffer.at(-1).keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            resultCsv << prevResultRow.str() << '\n';
        }
        prevResultRow.str({});
        prevResultRow << resultRow.str();
        resultRow.str({});
    } // eof loop over all images
    resultCsv << prevResultRow.str() << 0;
    resultCsv.close();
    return 0;
}
