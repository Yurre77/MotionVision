#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    Mat image, grayImage, edges;
    
    // Load the image of the connector with wires
    image = imread("c:\\Image00.jpg", IMREAD_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image." << endl;
        return -1;
    }

    // Convert to grayscale to simplify processing
    cvtColor(image, grayImage, COLOR_BGR2GRAY);

    // Apply Gaussian Blur to reduce noise and enhance edge detection
    GaussianBlur(grayImage, grayImage, Size(5, 5), 0);

    // Use Canny edge detection to find edges of the wires
    Canny(grayImage, edges, 50, 150, 3);

    // Find contours of the edges
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Filter contours based on their shape and size (assuming wires are long and thin)
    vector<vector<Point>> validContours;
    for (size_t i = 0; i < contours.size(); i++) {
        // Calculate the bounding rectangle of each contour
        Rect boundingBox = boundingRect(contours[i]);
        
        // Check if the contour could be a wire: long and narrow
        if (boundingBox.height > boundingBox.width * 2 && boundingBox.height > 50) {
            validContours.push_back(contours[i]);
        }
    }

    // Check if we detected exactly 4 wires
    if (validContours.size() == 4) {
        cout << "4 wires detected, all wires are connected!" << endl;
    } else {
        cout << validContours.size() << " wires detected, check connection." << endl;
    }

    // Draw the valid contours (wires) on the image
    for (size_t i = 0; i < validContours.size(); i++) {
        drawContours(image, validContours, (int)i, Scalar(0, 255, 0), 2, 8, hierarchy, 0);
    }

    // Display the original image with detected wires highlighted
    namedWindow("Wires Detected", WINDOW_AUTOSIZE);
    imshow("Wires Detected", image);

    // Wait for a key press
    waitKey(0);

    return 0;
}
