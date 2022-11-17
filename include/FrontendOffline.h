#include <vector>
#include <string>
#include <iostream>
#include <utility>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "Converter.h"
#ifndef FRONTENDOFFLINE_H
#define FRONTENDOFFLINE_H

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
    Feature(long unsigned int mapPointId, KeyPoint keyPoint, float depth, float uRight) : 
        mapPointId(mapPointId), keyPoint(keyPoint), depth(depth), uRight(uRight) {}


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
    std::pair<uint32_t, uint32_t> timestamp;
    long unsigned int frameId;
    // mapPointId, KeyPoint in this->frameId
    Mat pose;
    vector<Feature> features;

    FeatureTrack();
    FeatureTrack(const std::pair<uint32_t, uint32_t> &timestamp, const long unsigned int frameId, const Mat& pose) :
        timestamp(timestamp), frameId(frameId), pose(pose) {}

    void to_file(string path, char delimiter) {
        const size_t dimTranslation = 3, dimRotation = 4;
        ofstream ofp;
        ofp.open(path, ios::trunc);
        if (!ofp.is_open()) {
            perror(("[FeatureTrack] cannot open file " + path).c_str());
            exit(1);
        }
        ofp << frameId << endl;
         Mat pose_inv = pose.inv();
        Mat R = pose_inv.rowRange(0,3).colRange(0,3);
        Mat t = pose_inv.rowRange(0,3).col(3);
        vector<float> q = Converter::toQuaternion(R);
        ofp << setprecision(3);
        for (size_t i = 0; i < dimTranslation; ++i) { ofp << t.at<float>(i) << delimiter; }
        for (size_t i = 0; i < dimRotation; ++i) { ofp << q[i] << delimiter; }
        ofp << endl;
        for (Feature feature : features) {
            ofp << feature.to_str(delimiter) << endl;
        }
        ofp.close();
    }
    
};

struct MotionTrack {
    std::pair<uint32_t, uint32_t> timestamp; // timestamp correspond to current frame id
    long unsigned int frameId; // current frame id
    Mat velocity; // current_pose = velocity * last_pose
    MotionTrack();
    MotionTrack(const std::pair<uint32_t, uint32_t> &timestamp, const long unsigned int frameId, const Mat& velocity) :
        timestamp(timestamp), frameId(frameId), velocity(velocity) {}
    // filename: velocities/<frameId>.txt
    // format: <frameId> \n <translation quat_rotation> \n matrix.rowRange(0,3).colRange(0,4);
    void to_file(string path, char delimiter) {
        const size_t dimTranslation = 3, dimRotation = 4;
        ofstream ofp;
        ofp.open(path, ios::trunc);
        if (!ofp.is_open()) {
            perror(("[MotionTrack] cannot open file " + path).c_str());
            exit(1);
        }
        ofp << frameId << endl;
        Mat velocity_inv = velocity.inv();
        Mat R = velocity_inv.rowRange(0,3).colRange(0,3);
        Mat t = velocity_inv.rowRange(0,3).col(3);
        vector<float> q = Converter::toQuaternion(R);
        ofp << setprecision(3);
        for (size_t i = 0; i < dimTranslation; ++i) { ofp << t.at<float>(i) << delimiter; }
        for (size_t i = 0; i < dimRotation; ++i) { ofp << q[i] << delimiter; }
        ofp << endl;
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                ofp << velocity_inv.at<float>(i,j) << delimiter;
            }
        }
        ofp << endl;
        ofp.close();
    }
};

}

#endif // FRONTENDOFFLINE_H
