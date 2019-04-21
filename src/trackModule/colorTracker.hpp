#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

using cv::VideoCapture;
using cv::Mat;
using cv::Point;
using cv::Point2d;
using std::vector;


struct colorRange {
    cv::Scalar low;
    cv::Scalar high;
};

const colorRange rangeRed = { cv::Scalar(160,200,50) , cv::Scalar(200,255,255) };
const colorRange rangeBlue = { cv::Scalar(100,50,50) , cv::Scalar(140,255,255) };
const colorRange rangeGreen = { cv::Scalar(40,50,50) , cv::Scalar(80,255,255) };


class ColorTracker{
private:
    int cameraId;
    VideoCapture cap;
    int width;
    int height;
public:

    ColorTracker(int cameraId) noexcept(false);//throw std::exception
    ~ColorTracker() noexcept;

    int getWidth() noexcept;
    int getHeight() noexcept;


    void showCameraStatus();
    void showCaptureImage();
    Mat getCaptureImage() noexcept(false);//throw std::exception

    //Capture image and get center point of max area matched color region
    Point2d predict(const colorRange& range) noexcept(false);//throw std::exception
    Point2d predictShow(const colorRange& range) noexcept(false);//throw std::exception

    static void showImg(const Mat& img) noexcept(false);//throw std::exception

    //BGR2HSV
    static Mat getHSV(const Mat& img) noexcept(false);//throw std::exception
    //BGR2Hue
    static Mat getHue(const Mat& img) noexcept(false);//throw std::exception

    //BGR2mask
    static Mat getColorMask(const Mat& img,const colorRange& range) noexcept(false);//throw std::exception

    //get contours
    static vector<vector<Point>> getContours(const Mat& mask) noexcept(false);//throw std::exception

    //get convex contours
    static vector<vector<Point>> getConvexContours(const Mat& mask) noexcept(false);//throw std::exception

    //get maxArea contour
    static vector<Point> getMaxAreaContour(const vector<vector<Point>>& contours) noexcept(false);//throw std::exception
    
    //get center point
    static Point2d getCenterPoint(const vector<Point>& contour) noexcept(false);//throw std::exception
};