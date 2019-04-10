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

using namespace std;

Robot::Robot() {
    this->connected = true;
    this->robotInfo = {
            0,
            0,
            {0,0,0},
            {0,0,0},
            {0,0,0},
    };
    this->targetDuty = {0,0,0};
    this->nextCount = {0,0,0,0};
    this->prevCount = {0,0,0,0};
    this->mySerial = new serial::Serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    std::thread th(&Robot::communicate, this);
    sleep(300);
}

Robot::Robot(std::string& portPath) {
    this->connected = true;
    this->mySerial = new serial::Serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    this->targetDuty = {0,0,0};
    std::thread th(&Robot::communicate, this);
}


void Robot::communicate() {
    int i = 0;
    int delta = 1;
    std::string result;
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->writeDuty();
        result = this->mySerial->readline(24, "\n");
        i += delta;
        this->setDuty({i,i,i});
        if (i == 839) {
            delta = -1;
        } else if (i == -839) {
            delta = 1;
        }
        this->setCount(result);
        this->calcSpeed();
        std::cout << this->robotInfo.wheelVelocity[0] << std::endl;
    }
}

void Robot::calcSpeed() {
    std::vector<double> speed(3);
    int secDiff = this->nextCount[3] - this->prevCount[3] + TIM_MAX;
    double second = secDiff / 84.0 / 100000.0;
    for (int i = 0; i < 3; i++) {
        int diff = this->nextCount[i] - this->prevCount[i];
        if (diff <= 0) {
            diff += TIM_MAX;
            speed[i] = diff / COUNT_PER_ROTATE * 2 * 3.1415 * WHEEL_RADIUS / second;
        }
    }
    this->robotInfo.wheelVelocity = speed;
}

void Robot::setCount(std::string response) {
    vector<string> elems;
    string item;
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

void Robot::setDuty(const std::vector<int> vec) {
    this->targetDuty = vec;
}

bool Robot::getConnected() const {
    return this->connected;
}
