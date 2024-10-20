#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
    Mat image, greyimage;  // images are stored in datatype Mat (matrix)
    // Read the image
    image = imread("c:\\dice6.jpg", IMREAD_COLOR);
    if (!image.data){
        cout << "Could not open or find " << endl;
        return -1;
    }
    // Convert the image from color to gray values: 
    cvtColor(image, greyimage, COLOR_BGR2GRAY);
    // Reduce noise:
    GaussianBlur(greyimage, greyimage, Size(9, 9), 2, 2);
    // each element of the vector (array) circles is a vector of three float 
       // numbers: x-coordinate of the midpoint, y-coordinate of the midpoint, 
       // radius
    vector<Vec3f> circles;
    // Detect the circles with the Hough-algorithm:
    HoughCircles(greyimage, circles, HOUGH_GRADIENT, 2, 5, 100, 100, 0, 80);
    // Print (cout) the number of circles in the image 
    if (circles.size() >= 1) {
        cout << circles.size() << " circles detected!" << endl;
    } else {
        cout << "No circled detected." << endl;
    }
    // Print for each circle the coordinates of the centre and the radius 
    // example: circle[0][2]is the radius of circle number 0  
    int k;
    for (k = 0; k < circles.size(); k++) {
        cout << circles[k][0] << "    " << circles[k][1] << "   " << circles[k][2] << endl;
    }
    namedWindow("Dice", WINDOW_AUTOSIZE);
    imshow("Dice", image); 
    waitKey(0);
    return 0;
}