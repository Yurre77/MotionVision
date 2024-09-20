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
#define DESTY 1.0
#define LINK1 1.0
#define LINK2 1.0
#define LINK3 1.0
#define MAXTRIES 10
#define DAMPENING 0.1

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

    vector<double> returnV = {0.0, 0.0, 0.0};

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

    //calculate theta3
    vector<double> end_vector = {target[0] - virt_target[0], target[1] - virt_target[1], 0};
    vector<double> link3 = {LINK3, 0, 0};

    //angle calculations for theta3
    double cos_theta3 = dotProduct(link3, end_vector) / (calcVectorLen(link3) * calcVectorLen(end_vector));
    cross = crossProduct(link3, end_vector);
    double sin_theta3 = cross[2] / (calcVectorLen(link3) * calcVectorLen(end_vector));

    //determine if theta 3 should be positive or negative
    theta3 = acos(cos_theta3);
    if(sin_theta3 < 0){
        theta3 = -theta3;
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

    //pose matrix
    double p[3][4];
    //quaternions for fk calculations
    vector <double> q;
    //make it so that the pose matrix can be passed to the fk function
    double (*ptr)[4];
    ptr = p;

    //values to see if the target has been reached
    bool reached = false;
    int tries = 0;

    do{
        ik(theta1, theta2, theta3);

        //display IK results
        cout << "IK Joint Angles:" << endl;
        cout << "Theta1: " << theta1 * 180 / M_PI << " degrees" << endl;
        cout << "Theta2: " << theta2 * 180 / M_PI << " degrees" << endl;
        cout << "Theta3: " << theta3 * 180 / M_PI << " degrees" << endl;

        //joint angle vectors
        q = {theta1, theta2, theta3};

        //do fk calculations to verify if the ik calculations put the arm in the right position
        fk(q, ptr);

        // Display FK results
        cout << "FK End Effector Position:" << endl;
        cout << "(" << p[3][0] << ", " << p[3][1] << ")" << endl;

        // Check if FK matches target
        if (abs(p[3][0] - DESTX) < (1e-6 + DAMPENING) && abs(p[3][1] - DESTY) < (1e-6 + DAMPENING)) {
            cout << "IK solution validated!" << endl;
            reached = true;
        } else {
            cout << "IK solution validation failed! \n Trying again!" << endl;
            tries += 1;
        }
        if(tries > MAXTRIES){
            reached = true;
        }

    } while(!reached);

    return 0;
}