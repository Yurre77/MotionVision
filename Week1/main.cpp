#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

//function to calculate forward Kinematics (FK)
array<array<double, 3>, 4> fk(const array<double, 3>& q){
    //Arm links
    const double L1=1.0, L2=1.0, L3 = 1.0;

    //position vectors of each joint
    array <double, 3> p0 = {0.0, 0.0, 0.0};
    array <double, 3> p1;
    array <double, 3> p2;
    array <double, 3> p3;

    //calculations for forward kinematics

    //p1: pose of joint #2 (end of link 1)
    p1[0] = p0[0] + L1 * cos(q[0]);
    p1[1] = p0[1] + L1 * sin(q[0]);
    p1[2] = q[0];

    
    return {p0, p1, p2, p3};
}

int main(){
    //joint angle vectors
    double q[3] = {0.0, 0.2, 0.4};

    //pose matrix
    double p[3][4];



    return 0;
}