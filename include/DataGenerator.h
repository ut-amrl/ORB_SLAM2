#include <vector>
#include <string>
#include <iostream>
#include <utility>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "Converter.h"
#ifndef DATA_GENERATOR_H
#define DATA_GENERATOR_H

using namespace std;
using namespace cv;

namespace ORB_SLAM2 {

struct Feature {
    long unsigned int mapPointId;
    KeyPoint keyPoint;
    float depth;
    float uRight;
    KeyPoint rightKeyPoint;
    Feature() {
        uRight = -1;
    }
    Feature(long unsigned int mapPointId, KeyPoint keyPoint, float depth) : 
        mapPointId(mapPointId), keyPoint(keyPoint), depth(depth) {
        uRight = -1;
    }
    Feature(long unsigned int mapPointId, KeyPoint keyPoint, float depth, float uRight) : mapPointId(mapPointId), keyPoint(keyPoint), depth(depth), uRight(uRight) {}


    string to_str(char delimiter) {
        string ret = "";
        ret += to_string(mapPointId) + delimiter + to_string((float)keyPoint.pt.x) + delimiter + to_string((float)keyPoint.pt.y) + delimiter + to_string(depth);
        if (uRight < 0) {
            return ret;
        } else {
            ret += delimiter + to_string(uRight) + delimiter + to_string((float)keyPoint.pt.y);
            return ret;
        }
    }
};

struct FeatureTrack {
    double timestamp;
    long unsigned int frameId;
    // mapPointId, KeyPoint in this->frameId
    Mat Tcw;
    vector<Feature> features;

    FeatureTrack();
    FeatureTrack(const double timestamp, const long unsigned int frameId, const Mat Tcw) :
        timestamp(timestamp), frameId(frameId), Tcw(Tcw) {}

    void to_file(string path, char delimiter) {
        const size_t dimTranslation = 3, dimRotation = 4;
        ofstream ofp;
        ofp.open(path, ios::trunc);
        if (!ofp.is_open()) {
            perror("[FeatureTrack] cannot open file");
            exit(1);
        }
        // TODO: check for correctness; reference from System.cc:SaveTrajectoryTUM
        ofp << frameId << endl;
        Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        Mat twc = -Tcw.rowRange(0,3).col(3);
        vector<float> q = Converter::toQuaternion(Rwc);
        ofp << setprecision(3);
        for (size_t i = 0; i < dimTranslation; ++i) { ofp << twc.at<float>(i) << delimiter; }
        for (size_t i = 0; i < dimRotation; ++i) { ofp << q[i] << delimiter; }
        ofp << endl;
        for (Feature feature : features) {
            ofp << feature.to_str(delimiter) << endl;
        }
        ofp.close();
    }
    
};

}

#endif // OFFLINE_DATA_GENERATOR
