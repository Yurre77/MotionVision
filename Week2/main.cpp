/*
Student: Yurre Schram
Assignment: 2: Inverse kinematics
Date: 11-09-2024
*/

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

//constants
#define DEST {1.0, 2.0, 0}

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