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