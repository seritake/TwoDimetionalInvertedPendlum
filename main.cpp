#include <iostream>
#include "include/Robot.hpp"
#include "src/trackModule/cameraHandler.hpp"
#include <unistd.h>
#include <chrono>
#include <stdio.h>
#include <cmath>
#include <sys/time.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "vector"
#include <signal.h>

// For debug
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#define PRINT_MAT2(X, DESC) cout << DESC << ":\n" << X << endl << endl
#define PRINT_FNC    cout << "[" << __func__ << "]" << endl

using namespace std;
using namespace Eigen;

//constant values of objects(reference: balancing of a spherical inverted pendulum with an omni-directional mobile robot. Sho-Tsung Kao, Wan-Jung Chiou, and Ming-Tzu Ho)

const static double g = 9.8; //gravitational acceleration.

//the mobile robot
const static double M = 0.810; //(kg): mass of the mobile robot.
const static double L_c = 0.120; //(m): radius of the robot.
const static double I_czz = 1 / 2 * M * L_c * L_c; //(kg/m^{2}): moment of inertia of the robot about the Z-axis.

//the pendulum
const static double m = 0.300; //(kg): mass of the pendulum.
const static double l = 1.017; //(m): length of pendulum.
const static double I_p = 1.0 / 3.0 * m * l * l; //(kg/m^{2}): the moment of inertia of the pendulum.
//const double I_pxx = 3, I_pyy = 3, I_pzz = 3; //(kg/m^{2}): moment of inertia of the pendulum about the X-, Y-, Z-axis, respectively.

const static double M_t = M + m; //(kg): mass of the robot with pendulum.

//the motor
const static double K_t = 0.0917; //(kg*m/A) the motor torque constant.
const static double R_a = 6.67; //(Ω)the armature resistance of the motor.
const static double R_w = 0.0225; //(m): radius of the wheel.
const static double I_z =
        1.0 / 2.0 * 0.1 * R_w * R_w; //(kgm^{2}) the moment of inertia of the wheel around the driving shaft.

//Duty Ratio
const static double DUTY_MX = 839.0;
const static double DUTY_MULTI = DUTY_MX / 15.0;

//control
const static double nu_0 = 2.2; //need to be calculated.
const static double delta = 2.0;
const static double delta_1 = 2.0; //need to be calculated.
const static double a[2] = {-3.0 * K_t * K_t / (2 * R_w * R_w * M_t * R_a),
                            -3.0 * K_t * K_t * L_c * L_c / (R_w * R_w * I_z * R_a)};
const static double b[2] = {K_t / (2.0 * R_w * M_t * R_a), K_t * L_c / (R_w * I_z * R_a)};
const static double k_phi[2] = {0.5, 0.5}; //need to be changed.

// declared as global variable for signal handling.
Robot r;

template<typename T>
T saturate(T val, T mn, T mx) {
    T gradi = 100.0; //gradient of the saturation function.
    return min(max(gradi * val, mn), mx);
}

//i_p is either I_pxx or I_pyy.
template<typename T>
T cap_psi(T alpha, T d_alpha, T i_p) {
    return 1.0 + 2.0 * delta_1 * ((m * l * l) + i_p) / (m * l) * d_alpha * sin(alpha);
}

template<typename T>
T G_func(T alpha, T d_alpha, T i_p) {
    return g + (m * l * l + i_p) / (m * l * cos(alpha)) * d_alpha * d_alpha;
}

template<typename T>
T psi(T alpha, T i_p) {
    return -(m * l * l + i_p) / (m * l) * log((1 + tan(alpha / 2)) / (1 - tan(alpha / 2)));
}


/**
 * @voltCalculator
 * This function yields the voltage to contol inverted pendulum under the condition written in READ.me. 
 * @input (angle): This is composed of two param, first of which is angular displacement about y axis, second of which is that about x axis in the world coordinate.
 * @input (x): This is the position and the angle of the center of the omnidirectional mobile robot in the world coordinate. The third element is phi.
 * @input (v): velocity of the center of the omnidirectional mobile robot. The third is the velocity of phi.
 * @parameter (u): This is the value of the voltage to three servo motors.
 */

void voltCalculator(vector<int> &duty_ratio, vector<double> &angle, vector<double> &d_angle, vector<double> &x,
                    vector<double> &v) {
    Vector3d volt;
    double y_x, d_y_x, dd_y_x;
    double x_y, d_x_y, dd_x_y;
    double s_0[2], s_1[2], d_s_1[2];
    Vector3d u, f;
    Matrix3d A;

    double I_pxx = I_p * (1.0 - sin(angle[0])), I_pyy = I_p * (1.0 -
                                                               sin(angle[1])); //I_pzz = 1/12*m*(l*sqrt(sin(angle[0])*sin(angle[0]) + sin(angle[1])*sin(angle[1])))*(l*sqrt(sin(angle[0])*sin(angle[0]) + sin(angle[1])*sin(angle[1]))); //(kg/m^{2}): moment of inertia of the pendulum about the X-, Y-, Z-axis, respectively.
    y_x = x[0] - psi(angle[0], I_pyy);
    x_y = x[1] - psi(angle[1], I_pxx);


    d_y_x = v[0] + (m * l * l + I_pyy) / (m * l) * (1.0 + tan(angle[0] / 2.0) * tan(angle[0] / 2.0)) /
                   (1.0 - tan(angle[0] / 2.0) * tan(angle[0] / 2.0)) * d_angle[0];
    d_x_y = v[1] + (m * l * l + I_pxx) / (m * l) * (1.0 + tan(angle[1] / 2.0) * tan(angle[1] / 2.0)) /
                   (1.0 - tan(angle[1] / 2.0) * tan(angle[1] / 2.0)) * d_angle[1];

    //cout << d_y_x << '\t' << d_x_y << endl;

    dd_y_x = G_func(angle[0], d_angle[0], I_pyy) * tan(angle[0]);
    dd_x_y = G_func(angle[1], d_angle[1], I_pxx) * tan(angle[1]);

    //cout << dd_y_x << '\t' << dd_x_y << endl;

    s_1[0] = tan(angle[0]) + delta_1 * (y_x + d_y_x);
    s_1[1] = tan(angle[1]) + delta_1 * (x_y + d_x_y);

    //cout << "s_1:\t" <<  s_1[0] << '\t' << s_1[1] << endl;

    d_s_1[0] = 1 / (cos(angle[0]) * cos(angle[0])) + delta_1 * (d_y_x + dd_y_x);
    d_s_1[1] = 1 / (cos(angle[1]) * cos(angle[1])) + delta_1 * (d_x_y + dd_x_y);

    //cout << "d_s_1:\t" << d_s_1[0] << '\t' << d_s_1[1] << endl;

    s_0[0] = cos(angle[0]) * cos(angle[0]) * d_s_1[0] + delta * s_1[0];
    s_0[1] = cos(angle[1]) * cos(angle[1]) * d_s_1[1] + delta * s_1[1];

    f(0) = 1 / (m * l * cos(angle[0])) *
           (((m + M) * (m * l * l + I_pyy) - m * m * l * l * cos(angle[0]) * cos(angle[0])) *
            (nu_0 * saturate(s_0[0] * cap_psi(angle[0], d_angle[0], I_pyy), -1.0, 1.0)) -
            (m * m * l * l * d_angle[0] * d_angle[0]) * sin(angle[0]) * cos(angle[0]) +
            (m + M) * m * g * l * sin(angle[0]));


    f(1) = 1 / (m * l * cos(angle[1])) *
           (((m + M) * (m * l * l + I_pxx) - m * m * l * l * cos(angle[1]) * cos(angle[1])) *
            (nu_0 * saturate(s_0[1] * cap_psi(angle[1], d_angle[1], I_pxx), -1.0, 1.0)) -
            (m * m * l * l * d_angle[1] * d_angle[1]) * sin(angle[1]) * cos(angle[1]) +
            (m + M) * m * g * l * sin(angle[1]));

    f(0) = f(0) / M_t - a[0] * v[0];
    f(1) = f(1) / M_t - a[0] * v[1];

    f(2) = (-k_phi[0] * v[2] - a[1] * v[2] - k_phi[1] * x[2]) / (3 * b[1]);

    A << cos(x[2]) / (3 * b[0]), sin(x[2]) / (3 * b[0]), 1.0,
            (-sqrt(3) * sin(x[2]) - cos(x[2])) / (6 * b[0]), (sqrt(3) * cos(x[2]) - sin(x[2])) / (6 * b[0]), 1.0,
            (-sqrt(3) * sin(x[2]) + cos(x[2])) / (6 * b[0]), (-sqrt(3) * cos(x[2]) - sin(x[2])) / (6 * b[0]), 1.0;
    u = A * f;
    cout << "FORTH:" << u(0) << '\t' << u(1) << '\t' << u(2) << endl;

    cout << u(0) << endl;
    //change the vlotage to DT ratio.
    for (int i = 0; i <= 2; i++) {
        cout << u(i) * DUTY_MULTI << endl;
        duty_ratio[i] = u(i) * DUTY_MULTI;
    }
}

void exitHandler(int s) {
    printf("Caught signal %d\n",s);
    r.setDuty({0,0,0});
    exit(1);
}


int main() {
    r = Robot();
    vector<int> cameraList = {2, 1};//camerID 0 & 1
    vector<double> cameraAngle = {56, 56}; //camera's angle of view. specify for 2 cameras
    CameraHandler cameraHandler = CameraHandler(cameraList, cameraAngle);

    // handle SIGINT signal
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);


    struct timeval pre_time, now_time;
    gettimeofday(&pre_time, NULL);

    std::vector<double> position(3);
    std::vector<double> velocity(3);
    std::vector<double> angles(2);
    std::vector<double> pre_angles{0.0, 0.0};
    std::vector<double> d_angles(2);
    std::vector<int> duty_ratio(3);

    // expect to become faster when calling this function next time
    velocity = r.getVelocity();

    int wait;
    cout << "waiting for input...";
    cin >> wait;

    while (true) {

        position = r.getPosition();
        velocity = r.getVelocity();
        //Here get angles.
        angles = cameraHandler.getAngles();
        gettimeofday(&now_time, NULL);


        double elapsed = (now_time.tv_usec - pre_time.tv_usec) / 1000.0;
        for (int i = 0; i < 2; i++) {
            cout << "elapsed:" << '\t' << elapsed << endl;
            d_angles[i] = (angles[i] - pre_angles[i]) / elapsed;
            pre_angles[i] = angles[i];
            pre_time = now_time;
        }

        //cout << angles[0] << endl;
        voltCalculator(duty_ratio, angles, d_angles, position, velocity);

        cout << duty_ratio[0] << '\t' << duty_ratio[1] << '\t' << duty_ratio[2] << endl;
        for (int i = 0; i <= 2; i++) {
            if (duty_ratio[i] >= 800 || duty_ratio[i] <= -800) {
                cout << "DT Ratio is out of range.\n";
                duty_ratio[i] = 800 * (duty_ratio[i] > 0 ? 1 : -1);
            }
        }

        r.setDuty(duty_ratio);
    }
}
