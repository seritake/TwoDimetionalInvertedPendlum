//
// Created by 芹川武尊 on 2019/04/08.
//

#include "iostream"
#include "Robot.hpp"
#include "serial/serial.h"

Robot::Robot() {
    this->connected = true;
    serial::Serial mySerial("/dev/ ", 115200, serial::Timeout::simpleTimeout(1000));

    std::cout << "Is the serial port open?";
    if(mySerial.isOpen())
        std::cout << " Yes." << std::endl;
    else
        std::cout << " No." << std::endl;
}

bool Robot::getConnected() const {
    return this->connected;
}
