#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

//function to calculate forward Kinematics (FK)
vector <double> fk(const vector<double>& q){
    //Arm links
    const double L1=1.0, L2=1.0, L3 = 1.0;

    //position vectors of each joint
    vector <double> p0 = {0.0, 0.0, 0.0};
    vector <double> p1;
    vector <double> p2;
    vector <double> p3;

    //calculations for forward kinematics

    //p1: pose of joint #2 (end of link 1)
    p1[0] = p0[0] + L1 * cos(q[0]);
    p1[1] = p0[1] + L1 * sin(q[0]);
    p1[2] = q[0];

    //p2: pose of joint #3 (end of link 2)
    p2[0] = p1[0] + L2 * cos(q[0] + q[1]);
    p2[1] = p1[1] + L2 * sin(q[0] + q[1]);
    p2[2] = q[1];

    //p3: pose of the tool end (end of link 3)
    p3[0] = p2[0] + L3 * cos(q[0] + q[1] + q[2]);
    p3[1] = p2[1] + L3 * sin(q[0] + q[1] + q[2]);
    p3[2] = p2[2] + L3 * sin(q[1] + q[2]);

    return p0, p1, p2, p3;
}

int main(){
    //joint angle vectors
    vector <double> q = {0.1, 0.4, 0.9};

    //pose matrix
    double p[3][4];

    //solve kinematics
    p = fk(q);

    return 0;
}