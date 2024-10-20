#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath> // for calculating angles

using namespace cv;
using namespace std;

// Function to calculate the angle of a line given its start and end points
double calculateAngle(Point pt1, Point pt2) {
    double deltaY = pt2.y - pt1.y;
    double deltaX = pt2.x - pt1.x;
    return atan2(deltaY, deltaX) * 180.0 / CV_PI; // Convert radians to degrees
}

int main(int argc, char** argv) {
    Mat image, grayImage, edges;

    // Load the image of the metal pins
    image = imread("c:\\Pins00.jpg", IMREAD_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image." << endl;
        return -1;
    }

    // Convert the image to grayscale for easier processing
    cvtColor(image, grayImage, COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise and improve edge detection
    GaussianBlur(grayImage, grayImage, Size(5, 5), 0);

    // Use Canny edge detection to find edges
    Canny(grayImage, edges, 50, 150, 3);

    // Detect lines using the Hough Line Transform
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 50, 10);

    // Ideal vertical angle (in degrees) is 90
    const double idealVerticalAngle = 90.0;
    const double angleTolerance = 5.0; // Tolerance in degrees to consider the pin straight

    // Loop over the detected lines
    bool allPinsStraight = true;
    for (size_t i = 0; i < lines.size(); i++) {
        Point pt1(lines[i][0], lines[i][1]);
        Point pt2(lines[i][2], lines[i][3]);

        // Calculate the angle of the detected line (pin)
        double angle = calculateAngle(pt1, pt2);
        double angleDeviation = abs(idealVerticalAngle - abs(angle));

        // Check if the angle deviation is within the tolerance
        if (angleDeviation > angleTolerance) {
            allPinsStraight = false;
            // Draw bent pins in red
            line(image, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
        } else {
            // Draw straight pins in green
            line(image, pt1, pt2, Scalar(0, 255, 0), 2, LINE_AA);
        }

        // Output angle information for debugging
        cout << "Line " << i + 1 << ": Angle = " << angle << " degrees, Deviation = " << angleDeviation << " degrees" << endl;
    }

    // Final result output
    if (allPinsStraight) {
        cout << "All pins are straight!" << endl;
    } else {
        cout << "Some pins are bent!" << endl;
    }

    // Display the image with detected lines and color-coded straight/bent pins
    namedWindow("Detected Pins", WINDOW_AUTOSIZE);
    imshow("Detected Pins", image);

    // Wait for a key press
    waitKey(0);

    return 0;
}
