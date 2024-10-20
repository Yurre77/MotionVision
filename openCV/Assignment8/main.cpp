#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo.hpp>  // For non-local means denoising
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    Mat image, result;

    // Load the noisy image
    image = imread("c:\\300-JPEG-artifacts-original.bmp", IMREAD_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image." << endl;
        return -1;
    }

    // Create windows for displaying results
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("Filtered Image", WINDOW_AUTOSIZE);

    // Display original image
    imshow("Original Image", image);

     // Apply Gaussian Blurring to reduce Gaussian noise
    GaussianBlur(image, result, Size(5, 5), 0);
    cout << "Gaussian Blurring applied." << endl;

    // Display the filtered image
    imshow("Filtered Image", result);

    // Wait for a key press
    waitKey(0);

    return 0;
}
