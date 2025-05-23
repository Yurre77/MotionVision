#include <cmath>
#include <vector>
#include <iostream>

//constants
#define XTARGET 6
#define YTARGET 10

//robot limitations
#define LINK1 5.0
#define LINK2 5.0
#define LINK3 5.0
#define MINANGLE 1.22 //the smallest angle two links of the arm can have, -70 at 1.22
#define MAXANGLE 4.36 //the biggest angle two links of the arm can have, 250 at 4.36

//Calculation
#define MAXTRIES 10000
#define TOLERANCE 0.01
#define DAMPENING 0.05
#define DELTA 0.1

using namespace std;

//function to determine dot product
double dotProduct(vector<double> v1, vector<double> v2){
    double result;

    //convert the vectors to arrays, so that we can use each element seperately
    double V1[v1.size()]; 
    copy(v1.begin(), v1.end(), V1);
    double V2[v2.size()];
    copy(v2.begin(), v2.end(), V2);

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

//function to clip values
double clip(double n, double lower, double upper){
    return max(lower, min(n, upper));
}

class robotArm{
    public:
        double L1;
        double L2;
        double L3;

    double fk(double theta1, double theta2, double theta3){

        double x;
        double y;

        //needed FK calculations
        x = (L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3));
        y = (L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3));

        return x, y;
    }

    double ik(double x_target, double y_target){
        //a couple of variables for calculations
        double theta1, theta2, theta3 = 0;
        double x, y = 0;
        double error_x, error_y = 0;
        double x1, x2, x3, y1, y2, y3 = 0;
        vector<double> grad1, grad2, grad3 = {0, 0};
        double dot1, dot2, dot3 = 0;

        //loop variables
        int loops = 0;
        bool reached = false;

        do{
            x, y = fk(theta1, theta2, theta3);

            error_x = x_target - x;
            error_y = y_target - y;

            if(sqrt((error_x * error_x) + (error_y * error_y)) < TOLERANCE){
                reached = true;
            }

            x1, y1 = fk(theta1 + DELTA, theta2, theta3);
            x2, y2 = fk(theta1, theta2 + DELTA, theta3);
            x3, y3 = fk(theta1, theta2, theta3 + DELTA);

            //calculate vectors of every point in the arm to the desired position
            grad1 = vector<double>(x1 - x, y1 - y);
            grad2 = vector<double>(x2 - x, y2 - y);
            grad3 = vector<double>(x3 - x, y3 - y);

            //calculate dot products
            dot1 = dotProduct(grad1, vector<double>(error_x, error_y));
            dot2 = dotProduct(grad2, vector<double>(error_x, error_y));
            dot3 = dotProduct(grad3, vector<double>(error_x, error_y));

            //dampen the movements so that the arm does not break itself as fast
            theta1 += dot1 * DAMPENING;
            theta2 += dot2 * DAMPENING;
            theta3 += dot3 * DAMPENING;

            //limit the angles of each arm a little
            theta1 = clip(theta1, 0, M_PI);

            theta2 = clip(theta2, MINANGLE, MAXANGLE);
            theta3 = clip(theta3, MINANGLE, MAXANGLE);

            loops += 1;
        }while(loops < MAXTRIES && !(reached));

        cout << "IK stopped" << endl;
        cout << "loop count: " + loops << endl;
        
        return theta1, theta2, theta3;
    }
};

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