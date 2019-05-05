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
#include <eigen3/Eigen/LU>
#include "vector"
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <fstream>

// comment out this line if no logs are needed.
#define CREATE_LOG

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
const static double nu_0 = 5; //need to be calculated.
const static double delta = 5;
const static double delta_1 = 5; //need to be calculated.
const static double a[2] = {-3.0 * K_t * K_t / (2 * R_w * R_w * M_t * R_a),
                            -3.0 * K_t * K_t * L_c * L_c / (R_w * R_w * I_z * R_a)};
const static double b[2] = {K_t / (2.0 * R_w * M_t * R_a), K_t * L_c / (R_w * I_z * R_a)};
const static double k_phi[2] = {5, 5}; //need to be changed.

// back stepping control
const static double l_cog = 0.65;
const static double K1 = 10;
const static double K2 = 10;

// declared as global variable for signal handling.
Robot r;
#ifdef CREATE_LOG
std::ofstream fout;
#endif

vector<double> calcForce(std::vector<double>&& angle, std::vector<double>&& angle_velocity) {
    vector<double> e_b(2);
    e_b[0] = angle_velocity[0] + K1 * angle[0];
    e_b[1] = angle_velocity[1] + K1 * angle[1];

    vector<double> phi(2);
    phi[0] = m * angle_velocity[0] * angle_velocity[0] * sin(angle[0]) * cos(angle[0])
             / (-l_cog * (M + m * sin(angle[0]) * sin(angle[0])));
    phi[1] = m * angle_velocity[1] * angle_velocity[1] * sin(angle[1]) * cos(angle[1])
             / (-l_cog * (M + m * sin(angle[1]) * sin(angle[1])));

    vector<double> u(2);
    u[0] = -l_cog * ((K1 + K2) * e_b[0] + (1 - K1 * K1) * angle[0] + phi[0]);
    u[1] = -l_cog * ((K1 + K2) * e_b[1] + (1 - K1 * K1) * angle[1] + phi[1]);

    vector<double> f(2);
    f[0] = (M + m) * g * tan(angle[0]) - u[0] * (M + m * sin(angle[0]) * sin(angle[0])) / cos(angle[0]);
    f[1] = (M + m) * g * tan(angle[1]) - u[1] * (M + m * sin(angle[1]) * sin(angle[1])) / cos(angle[1]);

    return f;
}

vector<double> calcVoltage(vector<double> &&force, const Matrix3d &r_inv) {
    static Vector3d wheelForce;
    static Vector3d robotForce;
    robotForce << 0, force[0], force[1];
    wheelForce = r_inv * robotForce;
    return {wheelForce[0], wheelForce[1], wheelForce[2]};
}


void exitHandler(int s) {
    printf("Caught signal %d\n", s);
    r.setDuty({0, 0, 0});
    exit(1);
}

inline long getDiffUs(struct timeval &now, struct timeval &pre) {
    return (now.tv_sec - pre.tv_sec) * 1000000 + now.tv_usec - pre.tv_usec;
}

void calcJacob(Vector4d &x, double force, Matrix4d& Ad, double dt) {
    static double tmp;
    double c = cos(x[0]);
    double s = sin(x[0]);
    tmp = l_cog * (M + m) - l_cog * m * c * c;

    Ad << 1.0, dt, 0, 0,
            dt *
            (x[1] * x[1] * l_cog * l_cog * m * c / tmp + 2.0 * g * l_cog * l_cog * m * m * s * s * c * c / tmp / tmp
             + g * l * m * (s * s - c * c) / tmp -
             2.0 * l_cog * l_cog * m * (force + x[1] * x[1] * l * m * s) * s * c / tmp / tmp),
            2.0 * x[1] * dt * l_cog * l_cog * m * s / tmp + 1.0, 0, 0,
            0, 0, 1.0, 0,
            dt * (-x[1] * x[1] * l_cog * m * c * c / tmp - 2.0 * g * l_cog * m * (M + m) * s * s * c / tmp / tmp +
                  g * (M + m) * c / tmp
                  + 2.0 * l_cog * m * (force + x[1] * x[1] * l * m * s) * s * c * c / tmp / tmp +
                  (force + x[1] * x[1] * l * m * s) * s / tmp),
            -2.0 * x[1] * dt * l_cog * m * s * c / tmp, 0, 1.0;
}

void predictNextState(Vector4d &x, double force, Matrix4d &P, Matrix4d &Ad, Matrix4d &Q, double dt) {
    Vector4d x_old = x;
    double c = cos(x[0]);
    double s = sin(x[0]);
    double tmp = l_cog * (M + m) - l_cog * m * c * c;
    //cout << "force: " << force << endl;
    //cout << "x[1]:" << x[1] << endl;
    double tmp2 = force + x[1] * x[1] * l_cog * m * s;
    cout << "tmp2:" << tmp2 << endl;
    x[0] = x_old[1] * dt + x_old[0];
    x[1] = x_old[1] + dt * (-g * l_cog * m * s * c + l_cog * tmp2) / tmp;
    x[2] = x_old[2];
    x[3] = x_old[3] + dt * (g * (M + m) * s - tmp2 * c) / tmp;

    P = Ad * P * Ad.transpose() + Q;
}

void update(Vector4d& x, Matrix4d& P, Vector2d& y, Matrix<double, 2, 4>& Cd, Matrix2d& R) {
    Matrix<double, 4, 2> K = P * Cd.transpose() * (Cd * P * Cd.transpose() + R).inverse();
    x = x + K * (y - Cd * x);
    P = (MatrixXd::Identity(4,4) - K * Cd) - P;
}

int main() {
    //r = Robot();
    vector<int> cameraList = {2, 3, 1};//camerID 0 & 1
    vector<double> cameraAngle = {56, 56, 56}; //camera's angle of view. specify for 2 cameras
    CameraHandler cameraHandler = CameraHandler(cameraList, cameraAngle);

    // handle SIGINT signal
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

#ifdef CREATE_LOG
    // prepare for logs
    fout.open("../logs/log.csv");
    fout << "time,robot_position_x,robot_position_y,robot_position_w,robot_velocity_x,robot_velocity_y,robot_velocity_w,";
    fout << "angle_alpha,angle_beta,angle_dd_alpha,angle_dd_beta,dd_y_x,dd_x_y,s0_x,s0_y,s1_x,s1_y,f_x,f_y,u_1,u_2,u_3" << endl;

    struct timeval first_time;
    bool firstFlag = true;
#endif

    struct timeval pre_time, now_time;
    gettimeofday(&pre_time, NULL);

    std::vector<double> position(3);
    std::vector<double> velocity(3);
    Vector2d tmpPoint;
    std::vector<double> angles(2);
    std::vector<double> pre_angles{0.0, 0.0};
    std::vector<double> d_angles(2);
    std::vector<int> duty_ratio(3);
    Matrix2d rotMatrix;
    Vector2d anglesVec;
    std::vector<double> force = {0,0};

    Matrix4d Adx;
    Matrix4d Ady;
    double dt = 0.01;

    Matrix3d r_inv;
    r_inv << 25.0 / 9.0, 0, 2.0 / 3.0,
            25.0 / 9.0, -1.0 / 3.0 * 1.73205, -1.0 / 3.0,
            25.0 / 9.0, 1.0 / 3.0 * 1.73205, -1.0 / 3.0;

    Matrix4d Q;
    Matrix2d R;
    Q << 0.01, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0.001, 0,
            0, 0, 0, 0.1;
    R << 0.00001, 0, 0, 0.0001;

    Matrix<double, 2, 4> Cd;
    Cd << 1, 0, 1, 0, 0, 0, 0, 1;

    Matrix4d Px = Q;
    Matrix4d Py = Q;
    Vector4d x;
    Vector4d y;
    x << 0, 0, 0, 0;
    y << 0, 0, 0, 0;

    Vector2d output;

    // expect to become faster when calling this function next time
    angles = cameraHandler.getAngle();

    int wait;
    cout << "waiting for input...";
    cin >> wait;

    while (true) {
        calcJacob(x, force[0], Adx, dt);
        calcJacob(y, force[1], Ady, dt);
        //cout << "first" << endl;
        //PRINT_MAT(x);
        predictNextState(x, force[0], Px, Adx, Q, dt);
        //cout << "second" << endl;
        //PRINT_MAT(x);
        predictNextState(y, force[1], Py, Ady, Q, dt);
        gettimeofday(&now_time, NULL);
        dt = getDiffUs(now_time, pre_time) / 1000000.0;
        pre_time = now_time;

#ifdef CREATE_LOG
        static int count = 0;
        if (firstFlag) {
            first_time = now_time;
            firstFlag = false;
        }
        // using integer timestamp is often convenient when processing log
        fout << getDiffUs(now_time, first_time) << "," << position[0] << "," << position[1] << "," << position[2]
             << "," << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," << angles[0]*10 << "," << angles[1]*10 << ","
             << d_angles[0] << "," << d_angles[1] << ",";
        // todo: add input variables to log target
#endif
        //voltCalculator(duty_ratio, angles, d_angles, position, velocity);
        //vector<double> force_debug = calcForce(angles, d_angles);
        velocity = r.getVelocity();
        angles = cameraHandler.getAngle();
        output << angles[0], velocity[0];
        update(x, Px, output, Cd, R);
        //cout << "third" << endl;
        //PRINT_MAT(x);
        output << angles[1], velocity[1];
        update(y, Py, output, Cd, R);
        force = calcForce({x[0], y[0]}, {x[1], y[1]});
        for (int i = 0; i < 2; i++) {
            if (force[i] < -0.44 || force[i] > 0.44) {
                force[i] = 0.44 * (force[i] < 0 ? -1 : 1);
            }
        }
        vector<double> wheelForce = calcVoltage({force[0], force[1]}, r_inv);
        //vector<double> force = calcVoltage({2, 0}, r_inv);
        //cout << angles[0] << "\t" << angles[1] << endl;
        //cout << wheelForce[0] << "," << wheelForce[1] << "," << wheelForce[2] << endl;
        cout << "velocity: " << velocity[0] << "\t" << velocity[1] << endl;
        r.setForce(wheelForce);
    }
}