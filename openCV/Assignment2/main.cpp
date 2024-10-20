#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>  // for sqrt function to calculate distance

using namespace cv;
using namespace std;

// Function to calculate distance between two points
double calculateDistance(Point2f point1, Point2f point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

int main(int argc, char** argv) {
    Mat image, grayImage;

    // Load the image of the bracket
    image = imread("c:\\Bracket1.jpg", IMREAD_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image." << endl;
        return -1;
    }

    // Convert the image to grayscale
    cvtColor(image, grayImage, COLOR_BGR2GRAY);

    // Reduce noise with Gaussian blur
    GaussianBlur(grayImage, grayImage, Size(9, 9), 2, 2);

    // Vector to store detected circles (holes in the bracket)
    vector<Vec3f> circles;

    // Use Hough Circle Transform to detect holes (circular shapes)
    HoughCircles(grayImage, circles, HOUGH_GRADIENT, 1, grayImage.rows / 8, 100, 30, 10, 50);

    // Check if at least two holes are detected
    if (circles.size() >= 2) {
        cout << circles.size() << " holes detected!" << endl;

        // Draw the detected circles (holes) on the image
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            // Draw the circle center
            circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);

            // Draw the circle outline
            circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }

        // Assuming you want to calculate the distance between the first two detected holes
        Point2f center1(circles[0][0], circles[0][1]);
        Point2f center2(circles[1][0], circles[1][1]);

        // Calculate the Euclidean distance between the centers of the first two holes
        double distance = calculateDistance(center1, center2);

        // Output the distance
        cout << "Distance between holes: " << distance << " pixels" << endl;

    } else {
        cout << "Less than 2 holes detected. Distance calculation not possible." << endl;
    }

    // Display the image with detected holes and distance
    namedWindow("Detected Holes and Distance", WINDOW_AUTOSIZE);
    imshow("Detected Holes and Distance", image);

    // Wait for a key press
    waitKey(0);

    return 0;
}
