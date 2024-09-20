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
#define DEST {1.0, 2.0}
#define LINK1 1.0
#define LINK2 1.0
#define LINK3 1.0

//function to determine a dot product
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
void ik(const vector<double>& q, double (*p)[4], vector<double> desiredPos){
   
}

//function to calculate forward Kinematics (FK)
void fk(const vector<double>& q, double (*p)[4]){
    //Arm links
    const double L1=1.0, L2=1.0, L3 = 1.0;

    //position vectors of each joint
    *p[0] = 0.0, 0.0, 0.0;

    //calculations for forward kinematics
    //p1: pose of joint #2 (end of link 1)
    *p[1] = L1 * cos(q[0]), L1 * sin(q[0]), q[0];
    *p[1] = *p[0] + *p[1];

    //p2: pose of joint #3 (end of link 2)
    *p[2] = L2 * cos(q[0] + q[1]), L2 * sin(q[0] + q[1]), q[1];
    *p[2] = *p[1] + *p[2];

    //p3: pose of the tool end (end of link 3)
    *p[3] = L3 * cos(q[0] + q[1] + q[2]), L3 * sin(q[0] + q[1] + q[2]), q[2];
    *p[3] = *p[2] + *p[3];
}

int main(){
    //joint angle vectors
    vector <double> q = {0.1, 0.4, 0.9};

    //pose matrix
    double p[3][4];

    //make it so that the pose matrix can be passed to the ik function
    double (*ptr)[4];
    ptr = p;

    //do the ik calculations
    ik(q, ptr, DEST);

    return 0;
}