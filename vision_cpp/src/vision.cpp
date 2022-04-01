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

bool inside_rect(double x,double y,double x1,double y1,double x2,double y2)
{
    if((x > x1) && (x < x2) && (y > y1) && (y < y2)){
        return true;
    }
    else{
        return false;
    }
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
    int deviceID = 2;             // 0 = open default camera
    int apiID = CAP_ANY;
    cap.open(deviceID, apiID);

    //Get height and width of the picture
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout << "Width " << frame_width << "Height " << frame_height << "\n";

    int x1 = int(frame_width * 0.35);
    int y1 = int(frame_height * 0.3);
    int x2 = int(frame_width * 0.65);
    int y2 = int(frame_height * 0.5);

    Point start_point(x1, y1);
    Point end_point(x2, y2);


    //Record a video
    VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 60, Size(frame_width,frame_height));

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cout << "Start grabbing" << endl << "Press any key to terminate" << endl;
    while (cap.isOpened())
    {
        try
        {
            // wait for a new frame from camera and store it into 'frame'
            cap.read(frame);
            // check if we succeeded
            if (frame.empty()) {
                cerr << "ERROR! blank frame grabbed\n";
                break;
            }

            // the camera will be deinitialized automatically in VideoCapture destructor

            Mat bgr_image = frame.clone();

            Mat lab_image;
            cvtColor(bgr_image, lab_image, COLOR_BGR2Lab);

            // Extract the L channel
            vector<Mat> lab_planes(3);
            split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

            // apply the CLAHE algorithm to the L channel
            Ptr<CLAHE> clahe = createCLAHE();
            clahe->setClipLimit(2);
            clahe->setTilesGridSize(Size(8,8)); 
            Mat dst;
            clahe->apply(lab_planes[0], dst);

            // Merge the the color planes back into an Lab image
            dst.copyTo(lab_planes[0]);
            merge(lab_planes, lab_image);

            // convert back to RGB
            // Mat image_clahe;
            cvtColor(lab_image, bgr_image, COLOR_Lab2BGR);

            Mat orig_image = bgr_image.clone();

            medianBlur(bgr_image, bgr_image, 3);


            // Convert input image to HSV
            Mat hsv_image;
            cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);

            // Threshold the HSV image, keep only the red pixels
            Mat lower_red_hue_range;
            Mat upper_red_hue_range;
            inRange(hsv_image, Scalar(4.6, 218.79, 211.905), Scalar(3.95, 181, 171.12), lower_red_hue_range);
            inRange(hsv_image, Scalar(0.0, 150.0, 150.0), Scalar(15.0, 255.0, 255.0), upper_red_hue_range);

            // Combine the above two images
            Mat red_hue_image;
            addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);


            std::vector<Vec3f> circles;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            findContours(red_hue_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            int c_num = 0;

   
            vector<Point2f>centers(contours.size());
            vector<float>radius(contours.size());

    
        for (size_t i = 0; i < contours.size(); i++)
        {
            minEnclosingCircle(contours[i], centers[i], radius[i]);

            if (radius[i] > 15)
            {
                if ((radius[i] != 0) && (c_num==0))
                {
                    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                    circle(bgr_image, centers[i], (int)radius[i], color, 2);
                    rectangle(bgr_image, start_point, end_point, color, 2);
                    
                    bool inside = inside_rect(centers[i].x, centers[i].y, x1, y1, x2, y2);
                    
                    if (inside == 0){
                       
                        putText(bgr_image, "False", Point(frame_width -100 , 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);
                    }
                    else if(inside == 1){
                        putText(bgr_image, "True", Point(frame_width -100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, LINE_AA);
                    }
                    double estimated_length = depth_estimation(radius[i]);
                    putText(bgr_image, to_string(estimated_length), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, LINE_AA);
                    c_num+=1;
                }
            }

        }
        //Record a video of the output
        //video.write(bgr_image);
        imshow("img", bgr_image);
        if (waitKey(5) >= 0)
            break;
        }
        catch (int oof)
        {
            return 0;
        }
    }
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

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
