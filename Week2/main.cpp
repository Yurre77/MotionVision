#include <cmath>
#include <vector>
#include <iostream>

//constants
#define XTARGET 1
#define YTARGET 1
#define LINK1 1.0
#define LINK2 1.0
#define LINK3 1.0

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

        return x, y;
    }

    double ik(double x_target, double y_target){
        double theta1, theta2, theta3;
        double x, y;

        do{
            x, y = fk(theta1, theta2, theta3);
        }while(true);
        
        return theta1, theta2, theta3;
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

//function to calculate vector length
double calcVectorLen(vector<double> v){
    return (sqrt(((v[0]*v[0])+(v[1]*v[1])+(v[2]*v[2]))));
}

int main(){
    robotArm arm;

    arm.L1 = LINK1;
    arm.L2 = LINK2;
    arm.L3 = LINK3;

    double theta1, theta2, theta3 = arm.ik(XTARGET, YTARGET);

    //display IK results
    std::cout << "IK Joint Angles:" << endl;
    std::cout << "Theta1: " << theta1 * 180 / M_PI << " degrees" << endl;
    std::cout << "Theta2: " << theta2 * 180 / M_PI << " degrees" << endl;
    std::cout << "Theta3: " << theta3 * 180 / M_PI << " degrees" << endl;

    return 0;
}