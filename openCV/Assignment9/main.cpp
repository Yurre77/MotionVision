#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
    Mat image, greyimage;

    // Load the image of the dice
    image = imread("c:\\dice6.jpg", IMREAD_COLOR);
    if (!image.data){
        cout << "Could not open or find the image." << endl;
        return -1;
    }

    // Convert the image to grayscale
    cvtColor(image, greyimage, COLOR_BGR2GRAY);

    // Reduce noise in the image
    GaussianBlur(greyimage, greyimage, Size(9, 9), 2, 2);

    // Vector to store the detected circles
    vector<Vec3f> circles;

    // Apply the Hough Circle Transform to detect circles
    HoughCircles(greyimage, circles, HOUGH_GRADIENT, 1, greyimage.rows/8, 100, 30, 10, 40);

    // Output the number of circles detected (number of eyes on the dice)
    cout << "Number of eyes detected: " << circles.size() << endl;

    // Loop over the detected circles and draw them on the image
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // Draw the circle's center
        circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);

        // Draw the circle outline
        circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }

    // Display the original image with detected circles highlighted
    namedWindow("Detected Dice Eyes", WINDOW_AUTOSIZE);
    imshow("Detected Dice Eyes", image);

    // Wait indefinitely until a key is pressed
    waitKey(0);

    return 0;
}
