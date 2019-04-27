#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <future>
#include <exception>
#include "colorTracker.hpp"

using std::vector;
using std::cout;
using std::endl;
using std::exception;
using std::future;
using std::async;
using cv::Point2d;

//this class is not for general use.
class CameraHandler{
private:
    vector<ColorTracker> colorTrackers;
    vector<double> angles;
public:

    CameraHandler(const vector<int> cameraList,const vector<double> cameraAngle) noexcept(false);//throw std::exception

    vector<double> getAngles();//throw std::exception
};

CameraHandler::CameraHandler(const vector<int> cameraList,const vector<double> cameraAngle){
    this->colorTrackers.reserve(cameraList.size());
    for(int i = 0;i < cameraList.size();i++){
        this->colorTrackers.push_back(*new ColorTracker(cameraList[i]));
    }
    this->angles = cameraAngle;
}

vector<double> CameraHandler::getAngles(){
    vector<double> result;
    for (auto i = 0; i < this->colorTrackers.capacity(); i++) {
        try {
            auto point = this->colorTrackers[i].predictShow(rangeRed);
            auto width = this->colorTrackers[i].getWidth();
            //if (i == 0) cout << point.x << endl;
            result.push_back((point.x - width/2) * (this->angles[i]) / (width));
        } catch (exception& e){
            cout << e.what() << endl;
            result.push_back(0);
        }
    }


    /*vector<future<Point2d>> futures;
    for(auto i=0;i < this->colorTrackers.size();i++){
        futures.push_back(
           async( std::launch::async ,&ColorTracker::predict,&(this->colorTrackers[i]),rangeRed)
        );
    }

    vector<double> result;

    for(auto i = 0;i < futures.size();i++){
        try{
            auto point = futures[i].get();
            auto width = this->colorTrackers[i].getWidth();

            result.push_back((point.x - width/2) * (this->angles[i]) / (width));
        } catch (exception& e){
            cout << e.what() << endl;
            result.push_back(0);
        }
    }*/

    return result;
}