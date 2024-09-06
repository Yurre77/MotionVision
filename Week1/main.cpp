#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

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

    //make it so that the pose matrix can be passed to the fk function
    double (*ptr)[4];
    ptr = p;

    //do the fk calculations
    fk(q, ptr);

    //output the pose matrix
    for(const auto& pose : p){
        cout << "(" << pose[0] << ", " << pose[1] << ", " << pose[2] << ")" << endl;
    }

    return 0;
}