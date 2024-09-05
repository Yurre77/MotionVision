#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

#define JOINTS 3
#define MAX_TRIES 100

//make a class to keep track of positions
class Position{
    public:
        int x;
        int y;
};

//make a class to keep track of each segment
class Segment{
    public:
        int length;
        int angle;
        Position position;
};

int makeArm(Segment points[]){
    for(int i = 0; i < points->length; i++){
        points[i].position.x = 0;
        points[i].position.y = (0 + i);
    }
    return 0;
}

int main(){
    Segment points[JOINTS];

    makeArm(points);

    return 0;
}