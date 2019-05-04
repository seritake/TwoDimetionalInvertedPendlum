//
// Created by 芹川武尊 on 2019/04/08.
//

#pragma once
#include "serial/serial.h"
#include "vector"

const int TIM_MAX = 65536;
const double WHEEL_RADIUS = 22.0;
const int COUNT_PER_ROTATE = 480;

class Robot {
private:
    bool connected;
    serial::Serial* mySerial;
    std::vector<int> targetDuty;
    std::vector<int> prevCount;
    std::vector<int> nextCount;
    struct RobotInfo {
        double theta;
        double secDiff;
        std::vector<double> wheelVelocity;
        std::vector<double> robotVelocity;
        std::vector<double> selfCorVelocity;
        std::vector<double> position;
    } robotInfo;
    void communicate();
    void writeDuty();
    void calcSpeed();
    void calcVelocityAndTheta();
    void calcWorldVelocity();
    void calcPosition();
    void setCount(std::string& response);

public:
    Robot();
    Robot(std::string& portPath);
    bool getConnected() const;
    void setDuty(const std::vector<int>& vec);
    void setForce(const std::vector<double>& vec);
    const std::vector<double>& getVelocity() const;
    const std::vector<double>& getPosition() const;
};


