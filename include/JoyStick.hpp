#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <sys/time.h>
#include <unistd.h>
#include "vector"
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define JOYSTICK_DEV "/dev/input/js0"

struct joystick_state {
    std::vector<signed short> button;
    std::vector<signed short> axis;
};

struct joystick_position {
    float theta, r, x, y;
};

class cJoystick {
private:
    pthread_t thread;
    bool active;
    int joystick_fd;
    js_event *joystick_ev;
    joystick_state *joystick_st;
    __u32 version;
    __u8 axes;
    __u8 buttons;
    char name[256];

protected:
public:
    cJoystick();
    ~cJoystick();
    static void* loop(void* obj);
    void readEv();
    joystick_position joystickPosition(int n);
    bool buttonPressed(int n);
};