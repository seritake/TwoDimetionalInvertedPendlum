#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <future>
#include <exception>
#include "colorTracker.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include "vector"

#define FOCUS 400
#define CENTER_X 160
#define CENTER_Y 120
#define ROBOT_RADIUS 10
#define CAMERA_HEIGHT 4

// For debug
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#define PRINT_MAT2(X, DESC) cout << DESC << ":\n" << X << endl << endl
#define PRINT_FNC    cout << "[" << __func__ << "]" << endl

using namespace Eigen;


using std::vector;
using std::cout;
using std::endl;
using std::exception;
using std::future;
using std::async;
using cv::Point2d;
using namespace Eigen;

//this class is not for general use.
class CameraHandler {
private:
    vector<ColorTracker> colorTrackers;
    vector<double> angles;
public:

    CameraHandler(const vector<int> cameraList, const vector<double> cameraAngle) noexcept(false);//throw std::exception

    Vector3d getPoint();//throw std::exception
};

CameraHandler::CameraHandler(const vector<int> cameraList, const vector<double> cameraAngle) {
    this->colorTrackers.reserve(cameraList.size());
    for (int i = 0; i < cameraList.size(); i++) {
        this->colorTrackers.push_back(*new ColorTracker(cameraList[i]));
    }
    this->angles = cameraAngle;
}

Vector3d CameraHandler::getPoint() {
    Matrix<double, 6, 3> B;
    Vector6d b;
    vector<Point2d> points(3);
    for (auto i = 0; i < this->colorTrackers.capacity(); i++) {
        try {
            points[i] = this->colorTrackers[i].predict(rangeRed);
        } catch (exception &e) {
            return {0, 0, 100};
        }
    }
    b << CAMERA_HEIGHT * (CENTER_X - points[0].x), -FOCUS * ROBOT_RADIUS + CAMERA_HEIGHT * (CENTER_Y - points[0].y),
            CAMERA_HEIGHT * (CENTER_X - points[1].x), -FOCUS * ROBOT_RADIUS + CAMERA_HEIGHT * (CENTER_Y - points[1].y),
            CAMERA_HEIGHT * (CENTER_X - points[2].x), -FOCUS * ROBOT_RADIUS + CAMERA_HEIGHT * (CENTER_Y - points[2].y);
    B << 0, -FOCUS, -CENTER_X + points[0].x,
            FOCUS, 0, -CENTER_Y + points[0].y,
            1.73205 / 2 * FOCUS, 0.5 * FOCUS, -CENTER_X + points[1].x,
            -0.5 * FOCUS, 1.73205 / 2 * FOCUS, -CENTER_Y + points[1].y,
            -1.73205 / 2 * FOCUS, 0.5 * FOCUS, -CENTER_X + points[2].x,
            -0.5 * FOCUS, -1.73205 / 2 * FOCUS, -CENTER_Y + points[2].y;
    FullPivLU<Matrix3d> lu(B.transpose() * B);
    Vector3d x = lu.solve(-B.transpose() * b);
    //PRINT_MAT(x);

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
    return x;
    //{std::atan(x[0] / x[2]), std::atan(x[1] / x[2])};
}