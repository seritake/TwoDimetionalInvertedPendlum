#include <iostream>
#include "include/Robot.hpp"
#include "src/trackModule/cameraHandler.hpp"
#include <unistd.h>
#include <chrono>
#include <stdio.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "vector"

using namespace std;
using namespace Eigen;

//constant values of objects(reference: balancing of a spherical inverted pendulum with an omni-directional mobile robot. Sho-Tsung Kao, Wan-Jung Chiou, and Ming-Tzu Ho)

const static double g = 9.8; //gravitational acceleration.

//the mobile robot
const static double M = 0.810; //(kg): mass of the mobile robot.
const static double L_c = 0.120; //(m): radius of the robot.
const static double I_czz = 1/2*M*L_c*L_c; //(kg/m^{2}): moment of inertia of the robot about the Z-axis.

//the pendulum
const static double m = 0.200; //(kg): mass of the pendulum.
const static double l = 1.017; //(m): length of pendulum.
const static double I_p = 1/3*m*l*l; //(kg/m^{2}): the moment of inertia of the pendulum. 
//const double I_pxx = 3, I_pyy = 3, I_pzz = 3; //(kg/m^{2}): moment of inertia of the pendulum about the X-, Y-, Z-axis, respectively.

const static double M_t = M + m; //(kg): mass of the robot with pendulum.

//the motor
const static double K_t = 0.0917; //(kg*m/A) the motor torque constant.
const static double R_a = 6.67; //(Ω)the armature resistance of the motor.
const static double R_w = 0.0225; //(m): radius of the wheel.
const static double I_z = 0.02108; //(kgm^{2}) the moment of inertia of the wheel around the driving shaft.

//Duty Ratio
const static int DUTY_MX = 839;
const static double DUTY_MULTI = 15/DUTY_MX;

//control
const static double nu_0 = 2.2; //need to be calculated.
const static double delta = 0.5;
const static double delta_1 = 0.12; //need to be calculated.
const static double a[2] = {-3*K_t*K_t/(2*R_w*R_w*M_t*R_a), -3*K_t*K_t*L_c*L_c/(R_w*R_w*I_z*R_a)};
const static double b[2] = {K_t/(2*R_w*M_t*R_a), K_t*L_c/(R_w*I_z*R_a)};
const static double k_phi[2] = {0.5,0.5}; //need to be changed.

template<typename T>
T saturate(T val, T mn, T mx){
	T gradi = 1; //gradient of the saturation function.
	return min(max(gradi*val, mn), mx);
}

//i_p is either I_pxx or I_pyy.
template<typename T>
T cap_psi(T alpha, T d_alpha, T i_p){
	return 1 + 2*delta_1* ((m*l*l)+i_p) / (m*l) * d_alpha * sin(alpha);
}

template<typename T>
T G_func(T alpha, T d_alpha, T i_p){
	return g + (m*l*l + i_p) / (m*l*cos(alpha)) * d_alpha * d_alpha;
}

template<typename T>
T psi(T alpha, T i_p){
	return (m*l*l + i_p) / (m*l) * log( (1+tan(alpha/2)) / (1-tan(alpha/2)) );
}


/**
 * @voltCalculator
 * This function yields the voltage to contol inverted pendulum under the condition written in READ.me. 
 * @input (angle): This is composed of two param, first of which is angular displacement about y axis, second of which is that about x axis in the world coordinate.
 * @input (x): This is the position of the center of the omnidirectional mobile robot in the world coordinate.
 * @input (v): velocity of the center of the omnidirectional mobile robot.
 * @input (phi): orientation of the robot.
 * @input (v_phi): the velocity of phi.
 * @return (u): This is the value of the voltage to three servo motors.
 */

void voltCalculator(vector<int>& duty_ratio, vector<double>& angle, vector<double>& d_angle, vector<double>& x, vector<double>& v, double phi, double v_phi){
	Vector3d volt;
	double y_x, d_y_x, dd_y_x;
	double x_y, d_x_y, dd_x_y;
	double s_0[2], s_1[2], d_s_1[2];
	Vector3d u, f;
	Matrix3d A;

	double I_pxx = I_p*sin(angle[0]), I_pyy = I_p*sin(angle[1]); //I_pzz = 1/12*m*(l*sqrt(sin(angle[0])*sin(angle[0]) + sin(angle[1])*sin(angle[1])))*(l*sqrt(sin(angle[0])*sin(angle[0]) + sin(angle[1])*sin(angle[1]))); //(kg/m^{2}): moment of inertia of the pendulum about the X-, Y-, Z-axis, respectively.
	y_x = x[0] - psi(angle[0], I_pyy);
	x_y = x[1] - psi(angle[1], I_pxx);

	d_y_x = v[0] + (m*l*l + I_pyy) / (m*l*cos(angle[0])) * d_angle[0]; 
	d_x_y = v[1] + (m*l*l + I_pxx) / (m*l*cos(angle[1])) * d_angle[1];

	dd_y_x = G_func(angle[0], d_angle[0], I_pyy) * tan(angle[0]);
	dd_x_y = G_func(angle[1], d_angle[1], I_pxx) * tan(angle[1]);
	
	s_1[0] = tan(angle[0]) + delta_1*(y_x + d_y_x);
	s_1[1] = tan(angle[1]) + delta_1*(x_y + d_x_y);
	
	d_s_1[0] = 1/(cos(angle[0])*cos(angle[0])) + delta_1 * (d_y_x + dd_y_x);
	d_s_1[1] = 1/(cos(angle[1])*cos(angle[1])) + delta_1 * (d_x_y + dd_x_y);
	
	s_0[0] = cos(angle[0])*cos(angle[0])*d_s_1[0] + delta+s_1[0];
	s_0[1] = cos(angle[1])*cos(angle[1])*d_s_1[1] + delta+s_1[1];

	f(0) = 1/(m*l*cos(angle[0])) * ( ( (m+M)*(m*l*l+I_pyy)-m*m*l*l*cos(angle[0])*cos(angle[0]) )*(nu_0*saturate(s_0[0]*cap_psi(angle[0], d_angle[0], I_pyy),-1.0,1.0)) - (m*m*l*l*d_angle[0]*d_angle[0])*sin(angle[0])*cos(angle[0]) + (m+M)*m*g*l*sin(angle[0]) );
	
	f(1) = 1/(m*l*cos(angle[1])) * ( ( (m+M)*(m*l*l+I_pxx)-m*m*l*l*cos(angle[1])*cos(angle[1]) )*(nu_0*saturate(s_0[1]*cap_psi(angle[1], d_angle[1], I_pxx),-1.0,1.0)) - (m*m*l*l*d_angle[1]*d_angle[1])*sin(angle[1])*cos(angle[1]) + (m+M)*m*g*l*sin(angle[1]) );

	f(0) = f(0)/M_t - a[0]*v[0];
	f(1) = f(1)/M_t - a[0]*v[1];

	f(2) = (-k_phi[0]*v_phi - a[1]*v_phi - k_phi[1]*phi)/(3*b[1]);

	A << cos(phi)/(3*b[0])                     ,sin(phi)/(3*b[0])                     , 1,
	     (-sqrt(3)*sin(phi)-cos(phi))/(6*b[0]) ,(-sqrt(3)*cos(phi)+sin(phi))/(6*b[0]) , 1,
		 (-sqrt(3)*sin(phi)+cos(phi))/(6*b[0]) ,(-sqrt(3)*cos(phi)-sin(phi))/(6*b[0]) , 1;
	
	u = A * f;

    //change the vlotage to DT ratio.
    for(int i=0; i<= 2; i++){
        duty_ratio[i] = int(u(i) * DUTY_MULTI);
    }
}


int main() {
    Robot r = Robot();
	vector<int> cameraList = {1,2};//cameraID 0 & 1
    vector<double> cameraAngle = {56, 56};//camera's angle of view. specify for 2 cameras
    CameraHandler cameraHandler = CameraHandler(cameraList,cameraAngle);

	std::chrono::system_clock::time_point  pre_time, now_time;
	pre_time = std::chrono::system_clock::now();

    std::vector<double> position(3);
    std::vector<double> velocity(3);
    std::vector<double> angles(2);
	std::vector<double> pre_angles{0.0, 0.0};
	std::vector<double> d_angles(2);
    std::vector<int> duty_ratio(3);
    
    while(true) {
        usleep(30000);
        position = r.getPosition();
        velocity = r.getVelocity();
        //Here get angles.
        angles = cameraHandler.getAngles();
		now_time = std::chrono::system_clock::now();
		
		double elapsed = std::chrono::duration_cast<std::chrono::seconds>(now_time-pre_time).count();
		for(int i=0;i<2;i++){
			d_angles[i] = (angles[i]-pre_angles[i])/elapsed;
			pre_angles[i] = angles[i];
			pre_time = now_time;
		}

        cout << angles[0] << endl;
        voltCalculator(duty_ratio, angles, d_angles, position, velocity, position[2], velocity[2]);
        
        for(int i=0; i <= 2 ; i++){
            if(duty_ratio[i] >= 800 || duty_ratio[i] <= -800){
                cout << "DT Ratio is out of range.\n";
                duty_ratio[i] = 800 * duty_ratio[i] > 0? 1: -1;
            }
        }

        r.setDuty(duty_ratio);
    }
}
