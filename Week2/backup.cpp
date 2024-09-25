#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

class robotArm{
    public:
        double L1;
        double L2;
        double L3;

    double fk(double theta1, double theta2, double theta3){

        double x;
        double y;

        x = (L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3));
        y = (L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3));

        return (x, y);
    }

    void ik(){
        
    }
};

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

int main(){
    robotArm arm;

    arm.L1;
    arm.L2;
    arm.L3;

    return 0;
}