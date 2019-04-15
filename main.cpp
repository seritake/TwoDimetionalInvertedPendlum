#include <iostream>
#include "include/Robot.hpp"
#include <unistd.h>
#include <stdio.h>

int main() {
    Robot r = Robot();
    int i = 0;
    int delta = 5;
    //sleep(3);
    while(true) {
        usleep(30000);
        r.setDuty({i,i,i});
        i += delta;
        if (i >= 800) {
            delta = -5;
        } else if (i <= -800) {
            delta = 5;
        }
    }
    return 0;
}