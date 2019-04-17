#include <iostream>
#include "include/Robot.hpp"
#include <unistd.h>
#include <stdio.h>
#include "vector"

int main() {
    Robot r = Robot();
    int i = 0;
    int delta = 5;
    while(true) {
        usleep(30000);
        r.setDuty({i,i,i});
        i += delta;
        if (i >= 800) {
            delta = -5;
        } else if (i <= -800) {
            delta = 5;
        }
        static std::vector<double> position(3);
        position = r.getPosition();
        std::cout << position[0] << ",\t" << position[1] << ",\t" << position[2] << std::endl;
    }
}