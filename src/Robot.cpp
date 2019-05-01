//
// Created by 芹川武尊 on 2019/04/08.
//

#include "Robot.hpp"
#include "serial/serial.h"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <fstream>

const double PI = 3.1415926535;
const double L = 0.12;
const std::vector<std::vector<double> > gMatrix = {
        {0.0, -1.0 / (2.0 * std::sin(PI/3)), 1.0 / (2.0 * std::sin(PI / 3))},
        {1 / (1 + std::cos(PI / 3)), -1.0 / (2.0 * ( 1.0 + std::cos(PI / 3))), -1.0 / (2.0 * ( 1.0 + std::cos(PI / 3)))},
        {std::cos(PI/3) / ( L * (1 + std::cos(PI / 3))), 1.0 / (2.0 * L * (1 + std::cos(PI / 3))), 1 / (2 * L * (1 + std::cos(PI / 3)))}
};

inline std::vector<std::vector<double> > bMatrix(double theta) {
    return {{std::cos(theta), -std::sin(theta), 0},
            {std::sin(theta), std::cos(theta), 0},
            {0, 0, 1}
    };
}

Robot::Robot() {
    this->connected = true;
    this->robotInfo = {
            - PI / 2,
            0,
            {0,0,0},
            {0,0,0},
            {0,0,0},
            {0,0,0},
    };
    this->targetDuty = {0,0,0};
    this->nextCount = {0,0,0,0};
    this->prevCount = {0,0,0,0};
    this->mySerial = new serial::Serial("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(1000));
    std::thread th(&Robot::communicate, this);
    th.detach();
}

Robot::Robot(std::string& portPath) {
    this->connected = true;
    this->robotInfo = {
            0,
            0,
            {0,0,0},
            {0,0,0},
            {0,0,0},
            {0,0,0},
    };
    this->targetDuty = {0,0,0};
    this->nextCount = {0,0,0,0};
    this->prevCount = {0,0,0,0};
    this->mySerial = new serial::Serial(portPath, 115200, serial::Timeout::simpleTimeout(1000));
    std::thread th(&Robot::communicate, this);
    th.detach();
}


void Robot::communicate() {
    std::string result;
    this->writeDuty();
    result = this->mySerial->readline(24, "\n");
    this->setCount(result);
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->writeDuty();
        result = this->mySerial->readline(24, "\n");
        this->setCount(result);
        this->calcSpeed();
        /*std::cout << this->robotInfo.wheelVelocity[0] << "\t" << this->robotInfo.wheelVelocity[1]
        << "\t" << this->robotInfo.wheelVelocity[2] << std::endl;*/
        this->calcVelocityAndTheta();
        this->calcWorldVelocity();
        this->calcPosition();
    }
}

inline int abs(int num) {
    if (num > 0) {
        return num;
    } else {
        return -num;
    }
}

void Robot::calcSpeed() {
    std::vector<double> speed(3);
    int secDiff = this->nextCount[3] - this->prevCount[3];
    if (secDiff >= 0) {
        secDiff += TIM_MAX;
    } else if (-secDiff > TIM_MAX / 2) {
        secDiff += TIM_MAX * 2;
    } else {
        secDiff += TIM_MAX;
    }
    double second = secDiff / 84.0 / 100000.0;
    for (int i = 0; i < 3; i++) {
        int diff = this->nextCount[i] - this->prevCount[i];
        if (abs(diff) > TIM_MAX / 2) {
            if (diff > 0){
                diff = -(TIM_MAX - diff);
            } else {
                diff = TIM_MAX + diff;
            }
        }
        speed[i] = (double)diff / (double)COUNT_PER_ROTATE * 2.0 * PI * (double)WHEEL_RADIUS / second / 1000;
    }
    speed[0] = -speed[0];
    double tmp = speed[1];
    speed[1] = speed[2];
    speed[2] = -tmp;
    this->robotInfo.secDiff = second;
    this->robotInfo.wheelVelocity = speed;
}

void Robot::setCount(std::string& response) {
    std::vector<std::string> elems;
    std::string item;
    for (char ch: response) {
        if (ch == ',') {
            if (!item.empty())
                elems.push_back(item);
            item.clear();
        }
        else {
            item += ch;
        }
    }
    if (!item.empty())
        elems.push_back(item);

    this->prevCount = this->nextCount;
    this->nextCount = {std::stoi(elems[0]), std::stoi(elems[1]), std::stoi(elems[2]), std::stoi(elems[3])};
}

void Robot::writeDuty() {
    std::ostringstream sout;
    sout << "s";
    for (int i = 0; i < 3; i++) {
        std::string sign = this->targetDuty[i] > 0 ? "+" : "-";
        int duty = this->targetDuty[i] > 0 ? this->targetDuty[i] : -this->targetDuty[i];
        sout << sign << std::setfill('0') << std::setw(3) << duty;
        if (i != 2) {
            sout << ",";
        } else {
            sout << "e";
        }
    }
    std::string s = sout.str();
    this->mySerial->write(s);
}

void Robot::calcVelocityAndTheta() {
    std::vector<double> ans(3);
    for (int i = 0; i < 3; i++) {
        ans[i] = 0;
        for (int j = 0; j < 3; j++) {
            ans[i] += gMatrix[i][j] * this->robotInfo.wheelVelocity[j];
        }
    }
    this->robotInfo.selfCorVelocity = ans;
    this->robotInfo.theta += ans[2] * (double)this->robotInfo.secDiff;
}

void Robot::calcWorldVelocity() {
    std::vector<double> ans(3);
    std::vector<std::vector<double> > b = bMatrix(this->robotInfo.theta);
    for (int i = 0; i < 3; i++) {
        ans[i] = 0;
        for (int j = 0; j < 3; j++) {
            ans[i] += b[i][j] * this->robotInfo.selfCorVelocity[j];
        }
    }
    this->robotInfo.robotVelocity = ans;
}

void Robot::calcPosition() {
    std::vector<double> ans(3);
    for (int i = 0; i < 3; i++) {
        ans[i] = this->robotInfo.robotVelocity[i] * this->robotInfo.secDiff + this->robotInfo.position[i];
    }
    this->robotInfo.position = ans;
}

void Robot::setDuty(const std::vector<int>& vec) {
    this->targetDuty[0] = vec[0];
    this->targetDuty[1] = vec[2];
    this->targetDuty[2] = vec[1];
}

const std::vector<double>& Robot::getVelocity() const {
    return this->robotInfo.robotVelocity;
}

const std::vector<double>& Robot::getPosition() const {
    return this->robotInfo.position;
}

bool Robot::getConnected() const {
    return this->connected;
}
