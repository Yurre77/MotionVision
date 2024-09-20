/*
Student: Yurre Schram
Assignment: 2: Inverse kinematics
Date: 11-09-2024
*/

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;
using Vec3D = vector<double>;

//constants
#define DESTX 1.0 
#define DESTY 2.0
#define LINK1 1.0
#define LINK2 1.0
#define LINK3 1.0

//function to determine dot product
double dotProduct(vector<double> V1, vector<double> V2){
    double result;

    double Ax = V1[0] * V2[0];
    double Ay = V1[1] * V2[1];
    double Az = V1[2] * V2[2];

    result = Ax + Ay + Az;

    return result;
}

//function to calculate cross product
vector<double> crossProduct(vector<double> v1, vector<double> v2){

    vector<double> returnV;

    //calculate the return vector
    returnV[0] = v1[1] * v2[2] - v1[2] * v2[1];
    returnV[1] = v1[2] * v2[0] - v1[0] * v2[2];
    returnV[2] = v1[0] * v2[1] - v1[1] * v2[0];

    return returnV;
}

//function to calculate vector length
double calcVectorLen(vector<double> v){
    return (sqrt(((v[0]*v[0])+(v[1]*v[1])+(v[2]*v[2]))));
}

//function for the inwards kinematics
void ik(double &theta1, double &theta2, double &theta3){
    //calculate whether the robot arm is long enough to reach the destination
    double d = sqrt(DESTX * DESTX + DESTY * DESTY);

    if(d > LINK1 + LINK2 + LINK3 || d < fabs(LINK1 - LINK2 - LINK3)){
        cout << "Robot won't be able to reach the target" << endl;
        return;
    }

    //determine the targets that the robot should try to reach
    vector<double> target = {DESTX, DESTY, 0};
    vector<double> virt_target = {DESTX - LINK3 * (DESTX / d), DESTY - LINK3 * (DESTY / d), 0};

    //calculate theta2
    vector<double> u1 = {LINK1, 0, 0};
    vector<double> u2 = {virt_target[0] - u1[0], virt_target[1] - u1[1], 0};

    //angle calculations for theta2
    double cos_theta2 = dotProduct(u1, u2) / calcVectorLen(u1) * calcVectorLen(u2);
    vector<double> cross = crossProduct(u1, u2);
    double sin_theta2 = cross[2] / (calcVectorLen(u1) * calcVectorLen(u2));

    //determine whether theta2 should be positive or negative
    theta2 = acos(cos_theta2);
    if(sin_theta2 < 0){
        theta2 = -theta2;
    }

    //calculate theta1
    vector<double> base_target = virt_target;
    vector<double> base_x = {1.0, 0.0, 0.0};

    //angle calculations for theta1
    double cos_theta1 = dotProduct(base_x, base_target) / calcVectorLen(base_target);
    cross = crossProduct(base_x, base_target);
    double sin_theta1 = cross[2] / calcVectorLen(base_target);

    //determine whether theta1 is positive or negative
    theta1 = acos(cos_theta1);
    if(sin_theta1 < 0){
        theta1 = -theta1;
    }

    
}

//function to calculate forward Kinematics (FK)
void fk(const vector<double>& q, double (*p)[4]){
    //position vectors of each joint
    *p[0] = 0.0, 0.0, 0.0;

    //calculations for forward kinematics
    //p1: pose of joint #2 (end of link 1)
    *p[1] = LINK1 * cos(q[0]), LINK1 * sin(q[0]), q[0];
    *p[1] = *p[0] + *p[1];

    //p2: pose of joint #3 (end of link 2)
    *p[2] = LINK2 * cos(q[0] + q[1]), LINK2 * sin(q[0] + q[1]), q[1];
    *p[2] = *p[1] + *p[2];

    //p3: pose of the toolend (end of link 3)
    *p[3] = LINK3 * cos(q[0] + q[1] + q[2]), LINK3 * sin(q[0] + q[1] + q[2]), q[2];
    *p[3] = *p[2] + *p[3];
}

int main(){
    //variables to calculate the angles on
    double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0;

    //joint angle vectors
    vector <double> q = {theta1, theta2, theta3};
    //pose matrix
    double p[3][4];
    //make it so that the pose matrix can be passed to the fk function
    double (*ptr)[4];
    ptr = p;
    //do the ik calculations
    fk(q, ptr);

    return 0;
}