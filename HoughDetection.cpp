// HoughDetection.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <fstream>
#include <cmath>


using namespace cv;
using namespace std;

double depth_estimation(double r) {
    double depth = 1246.1 / (pow(r, 0.99));
    return depth;
}



int main(int argc, char** argv)
{

    Mat src_gray;
    int thresh = 100;
    RNG rng(12345);

    Mat canny_output;
    Canny(src_gray, canny_output, thresh, thresh * 2);


    Mat frame;
    Mat output;
    VideoCapture cap;
    int deviceID = 0;             // 0 = open default camera
    int apiID = CAP_ANY;
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cout << "Start grabbing" << endl << "Press any key to terminate" << endl;
    while(true)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }


    // the camera will be deinitialized automatically in VideoCapture destructor

        Mat bgr_image = frame;


        Mat orig_image = bgr_image.clone();

        medianBlur(bgr_image, bgr_image, 3);
        
        // Convert input image to HSV
        Mat hsv_image;
        cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);

        // Threshold the HSV image, keep only the red pixels
        Mat lower_red_hue_range;
        Mat upper_red_hue_range;
        inRange(hsv_image, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
        inRange(hsv_image, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);
        
        // Combine the above two images
        Mat red_hue_image;
        addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);


        

        std::vector<Vec3f> circles;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(red_hue_image,contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f>centers(contours.size());
        vector<float>radius(contours.size());

        Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
        for (size_t i = 0; i < contours.size(); i++)
        {
            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            drawContours(drawing, contours_poly, (int)i, color);
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
            circle(drawing, centers[i], (int)radius[i], color, 2);
        }
        imshow("Contours", drawing);
        waitKey(0);
    }





    //const char* filename = argc >= 2 ? argv[1] : "smarties.png";
    // Loads an image
    //Mat src = imread(samples::findFile(filename), IMREAD_COLOR);
    // Check if image is loaded fine
    /*if (src.empty()) {
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", filename);
        return EXIT_FAILURE;
    }*/

    /*Mat output;
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cout << "Could not initialize capturing... \n";
        return 0;
        }

    while (1) {
        cap >> output;
        

        Mat gray;
        cvtColor(output, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);
        vector<Vec3f> circles;
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
            gray.rows / 16,  // change this value to detect circles with different distances to each other
            100,30, 1, 70// change the last two parameters
       // (min_radius & max_radius) to detect larger circles
        );
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            // circle center
            circle(output, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            cout << radius << "\n";
            circle(output, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        }
        imshow("detected circles", output);
        char c = (char)waitKey(10);
        if (c == 27) break;
    }*/
    return EXIT_SUCCESS;

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
