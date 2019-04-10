//
// Created by 芹川武尊 on 2019/04/08.
//

#pragma once
#include "serial/serial.h"
#include "vector"

const int TIM_MAX = 65536;
const int WHEEL_RADIUS = 24;
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
        double omega;
        std::vector<double> wheelVelocity;
        std::vector<double> robotVelocity;
        std::vector<double> selfCorVelocity;
    } robotInfo;
    void updateRobotInfo() const;
    void communicate();
    void writeDuty();
    void calcSpeed();
    void setCount(std::string response);

public:
    Robot();
    Robot(std::string& portPath);
    bool getConnected() const;
    void sendCommand();
    void setDuty(const std::vector<int> vec);
};


