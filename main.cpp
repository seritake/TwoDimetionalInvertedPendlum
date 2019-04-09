#include <iostream>
#include "include/Robot.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;
    Robot r = Robot();
    std::cout << r.getConnected() << std::endl;
    return 0;
}