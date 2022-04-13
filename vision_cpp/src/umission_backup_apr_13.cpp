/***************************************************************************
 *   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"
#include "ulibpose2pose.h"

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <fstream>
#include <cmath>

using namespace cv;
using namespace std;

int DIM_X = 640;
int DIM_Y = 480;
extern VideoWriter video("outcpp.avi", v4l2_fourcc('M', 'J', 'P', 'G'), 2, Size(DIM_X, DIM_Y));

double depth_estimation(double r)
{
    double depth = 1246.1 / (pow(r, 0.99));
    return depth;
}

bool inside_rect(double x, double y, double x1, double y1, double x2, double y2)
{
    if ((x > x1) && (x < x2) && (y > y1) && (y < y2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool right_size(double depth, int min_interval = 30, int max_interval = 45)
{
    bool size = ((min_interval <= depth) && (depth <= max_interval));
    return size;
}

UMission::UMission(UBridge *regbot, UCamera *camera)
{
    cam = camera;
    bridge = regbot;

    threadActive = 100;
    // initialize line list to empty
    for (int i = 0; i < missionLineMax; i++)
    { // add to line list
        lines[i] = lineBuffer[i];
        // terminate c-strings strings - good practice, but not needed
        lines[i][0] = '\0';
    }
    // start mission thread
    th1 = new thread(runObj, this);
    //   play.say("What a nice day for a stroll\n", 100);
    //   sleep(5);
}

UMission::~UMission()
{
    printf("Mission class destructor\n");
}

void UMission::run()
{
    while (not active and not th1stop)
        usleep(100000);
    //   printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
    if (not th1stop)
        runMission();
    printf("UMission::run: mission thread ended\n");
}

void UMission::printStatus()
{
    printf("# ------- Mission ----------\n");
    printf("# active = %d, finished = %d\n", active, finished);
    printf("# mission part=%d, in state=%d\n", mission, missionState);
}

/**
 * Initializes the communication with the robobot_bridge and the REGBOT.
 * It further initializes a (maximum) number of mission lines
 * in the REGBOT microprocessor. */
void UMission::missionInit()
{ // stop any not-finished mission
    bridge->send("robot stop\n");
    // clear old mission
    bridge->send("robot <clear\n");
    //
    // add new mission with 3 threads
    // one (100) starting at event 30 and stopping at event 31
    // one (101) starting at event 31 and stopping at event 30
    // one (  1) used for idle and initialisation of hardware
    // the mission is started, but staying in place (velocity=0, so servo action)
    //
    bridge->send("robot <add thread=1\n");
    // Irsensor should be activated a good time before use
    // otherwise first samples will produce "false" positive (too short/negative).
    bridge->send("robot <add irsensor=1,vel=0:dist<0.2\n");
    //
    // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
    bridge->send("robot <add thread=100,event=30 : event=31\n");
    for (int i = 0; i < missionLineMax; i++)
        // send placeholder lines, that will never finish
        // are to be replaced with real mission
        // NB - hereafter no lines can be added to these threads, just modified
        bridge->send("robot <add vel=0 : time=0.1\n");
    //
    bridge->send("robot <add thread=101,event=31 : event=30\n");
    for (int i = 0; i < missionLineMax; i++)
        // send placeholder lines, that will never finish
        bridge->send("robot <add vel=0 : time=0.1\n");
    usleep(10000);
    //
    //
    // send subscribe to bridge
    bridge->pose->subscribe();
    bridge->edge->subscribe();
    bridge->motor->subscribe();
    bridge->event->subscribe();
    bridge->joy->subscribe();
    bridge->motor->subscribe();
    bridge->info->subscribe();
    bridge->irdist->subscribe();
    bridge->imu->subscribe();
    usleep(10000);
    // there maybe leftover events from last mission
    bridge->event->clearEvents();
}

void UMission::sendAndActivateSnippet(char **missionLines, int missionLineCnt)
{
    // Calling sendAndActivateSnippet automatically toggles between thread 100 and 101.
    // Modifies the currently inactive thread and then makes it active.
    const int MSL = 100;
    char s[MSL];
    int threadToMod = 101;
    int startEvent = 31;
    // select Regbot thread to modify
    // and event to activate it
    if (threadActive == 101)
    {
        threadToMod = 100;
        startEvent = 30;
    }
    if (missionLineCnt > missionLineMax)
    {
        printf("# ----------- error - too many lines ------------\n");
        printf("# You tried to send %d lines, but there is buffer space for %d only!\n", missionLineCnt, missionLineMax);
        printf("# set 'missionLineMax' to a higher number in 'umission.h' about line 57\n");
        printf("# (not all lines will be send)\n");
        printf("# -----------------------------------------------\n");
        missionLineCnt = missionLineMax;
    }
    // send mission lines using '<mod ...' command
    for (int i = 0; i < missionLineCnt; i++)
    { // send lines one at a time
        if (strlen((char *)missionLines[i]) > 0)
        { // send a modify line command
            snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i + 1, missionLines[i]);
            bridge->send(s);
        }
        else
            // an empty line will end code snippet too
            break;
    }
    // let it sink in (10ms)
    usleep(10000);
    // Activate new snippet thread and stop the other
    snprintf(s, MSL, "<event=%d\n", startEvent);
    bridge->send(s);
    // save active thread number
    threadActive = threadToMod;
}

//////////////////////////////////////////////////////////

/**
 * Thread for running the mission(s)
 * All missions segments are called in turn based on mission number
 * Mission number can be set at parameter when starting mission command line.
 *
 * The loop also handles manual override for the gamepad, and resumes
 * when manual control is released.
 * */
void UMission::runMission()
{ /// current mission number
    mission = fromMission;
    int missionOld = mission;
    bool regbotStarted = false;
    /// end flag for current mission
    bool ended = false;
    /// manuel override - using gamepad
    bool inManual = false;
    /// debug loop counter
    int loop = 0;
    // keeps track of mission state
    missionState = 0;
    int missionStateOld = missionState;
    // fixed string buffer
    const int MSL = 120;
    char s[MSL];
    /// initialize robot mission to do nothing (wait for mission lines)
    missionInit();
    /// start (the empty) mission, ready for mission snippets.
    bridge->send("start\n"); // ask REGBOT to start controlled run (ready to execute)
    bridge->send("oled 3 waiting for REGBOT\n");
    //   play.say("Waiting for robot data.", 100);
    ///
    for (int i = 0; i < 3; i++)
    {
        if (not bridge->info->isHeartbeatOK())
        { // heartbeat should come at least once a second
            sleep(2);
        }
    }
    if (not bridge->info->isHeartbeatOK())
    { // heartbeat should come at least once a second
        // play.say("Oops, no usable connection with robot.", 100);
        //    system("espeak \"Oops, no usable connection with robot.\" -ven+f4 -s130 -a60 2>/dev/null &");
        bridge->send("oled 3 Oops: Lost REGBOT!");
        printf("# ---------- error ------------\n");
        printf("# No heartbeat from robot. Bridge or REGBOT is stuck\n");
        //     printf("# You could try restart ROBOBOT bridge ('b' from mission console) \n");
        printf("# -----------------------------\n");
        //
        if (false)
            // for debug - allow this
            stop();
    }
    /// loop in sequence every mission until they report ended
    while (not finished and not th1stop)
    { // stay in this mission loop until finished
        loop++;
        // test for manuel override (joy is short for joystick or gamepad)
        if (bridge->joy->manual)
        { // just wait, do not continue mission
            usleep(20000);
            if (not inManual)
            {
                //         system("espeak \"Mission paused.\" -ven+f4 -s130 -a40 2>/dev/null &");
                // play.say("Mission paused.", 90);
            }
            inManual = true;
            bridge->send("oled 3 GAMEPAD control\n");
        }
        else
        { // in auto mode
            if (not regbotStarted)
            { // wait for start event is received from REGBOT
                // - in response to 'bot->send("start\n")' earlier
                if (bridge->event->isEventSet(33))
                { // start mission (button pressed)
                    //           printf("Mission::runMission: starting mission (part from %d to %d)\n", fromMission, toMission);
                    regbotStarted = true;
                }
            }
            else
            { // mission in auto mode
                if (inManual)
                { // just entered auto mode, so tell.
                    inManual = false;
                    //           system("espeak \"Mission resuming.\" -ven+f4 -s130 -a40 2>/dev/null &");
                    // play.say("Mission resuming", 90);
                    bridge->send("oled 3 running AUTO\n");
                }
                switch (mission)
                {
                case 1: // running auto mission
                    ended = mission1(missionState);
                    break;
                default:
                    // no more missions - end everything
                    finished = true;
                    break;
                }
                if (ended)
                { // start next mission part in state 0
                    mission++;
                    ended = false;
                    missionState = 0;
                }
                // show current state on robot display
                if (mission != missionOld or missionState != missionStateOld)
                { // update small O-led display on robot - when there is a change
                    UTime t;
                    t.now();
                    snprintf(s, MSL, "oled 4 mission %d state %d\n", mission, missionState);
                    bridge->send(s);
                    if (logMission != NULL)
                    {
                        fprintf(logMission, "%ld.%03ld %d %d\n",
                                t.getSec(), t.getMilisec(),
                                missionOld, missionStateOld);
                        fprintf(logMission, "%ld.%03ld %d %d\n",
                                t.getSec(), t.getMilisec(),
                                mission, missionState);
                    }
                    missionOld = mission;
                    missionStateOld = missionState;
                }
            }
        }
        //
        // check for general events in all modes
        // gamepad buttons 0=green, 1=red, 2=blue, 3=yellow, 4=LB, 5=RB, 6=back, 7=start, 8=Logitech, 9=A1, 10 = A2
        // gamepad axes    0=left-LR, 1=left-UD, 2=LT, 3=right-LR, 4=right-UD, 5=RT, 6=+LR, 7=+-UD
        // see also "ujoy.h"
        if (bridge->joy->button[BUTTON_RED])
        { // red button -> save image
            if (not cam->saveImage)
            {
                printf("UMission::runMission:: button 1 (red) pressed -> save image\n");
                cam->saveImage = true;
            }
        }
        if (bridge->joy->button[BUTTON_YELLOW])
        { // yellow button -> make ArUco analysis
            if (not cam->doArUcoAnalysis)
            {
                printf("UMission::runMission:: button 3 (yellow) pressed -> do ArUco\n");
                cam->doArUcoAnalysis = true;
            }
        }
        // are we finished - event 0 disables motors (e.g. green button)
        if (bridge->event->isEventSet(0))
        { // robot say stop
            finished = true;
            printf("Mission:: insist we are finished\n");
        }
        else if (mission > toMission)
        { // stop robot
            // make an event 0
            bridge->send("stop\n");
            // stop mission loop
            finished = true;
        }
        // release CPU a bit (10ms)
        usleep(1000);
    }
    bridge->send("stop\n");
    snprintf(s, MSL, "Robot %s finished.\n", bridge->info->robotname);
    //   system(s);
    // play.say(s, 100);
    printf("%s", s);
    bridge->send("oled 3 finished\n");
}

////////////////////////////////////////////////////////////

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission1(int &state)
{
    Mat src_gray;
    int thresh = 100;
    RNG rng(12345);

    Mat frame;
    Mat output;
    VideoCapture cap;
    string deviceID = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_9A2A20BF-video-index0"; // 0 = open default camera
    int apiID = CAP_V4L2;
    cap.open(deviceID, apiID);
    // printf("setup video capture\n");
    int pic_id = 0;

    // Get height and width of the picture
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    // cout << "Width " << frame_width << "\n"
    //      << "Height " << frame_height << "\n";

    int left_x = int(0.45 * DIM_X);
    int right_x = int(0.55 * DIM_X);

    Point start_point(left_x, 0);
    Point end_point(right_x, DIM_Y);

    if (!cap.isOpened())
    {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    // printf("reached while loop\n");

    bool finished = false;
    // First commands to send to robobot in given mission
    // (robot sends event 1 after driving 1 meter)):
    switch (state)
    {
    case 0:
        bridge->send("oled 5 press green to start");
        state = 10;
        break;
    case 1:
        // if (bridge->joy->button[BUTTON_GREEN])
        state = 5;
        break;
    case 10:
        // make sure event 1 is cleared
        bridge->event->isEventSet(1);
        if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < (2 * 180 / M_PI))
        { // finished first drive and turnrate is zero'ish
            state = 11;
            featureCnt = 0;
            // bool find_tennis_ball = True;
            // wait further 30ms - about one camera frame at 30 FPS
            usleep(1000);
        }
        break;
    case 11:
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty())
        {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // the camera will be deinitialized automatically in VideoCapture destructor
        // printf("Start\n");
        Mat bgr_image = frame.clone();

        Mat lab_image;
        cvtColor(bgr_image, lab_image, COLOR_BGR2Lab);

        // Extract the L channel
        vector<Mat> lab_planes(3);
        split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

        // apply the CLAHE algorithm to the L channel
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(2);
        clahe->setTilesGridSize(Size(8, 8));
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
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(red_hue_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        int c_num = 0;

        vector<Point2f> centers(contours.size());
        vector<float> radius(contours.size());

        if (contours.size() != 0)
        {
            for (size_t i = 0; i < contours.size(); i++)
            {
                // printf("found a contour\n");
                minEnclosingCircle(contours[i], centers[i], radius[i]);

                if (radius[i] > 15)
                {
                    if ((radius[i] != 0) && (c_num == 0))
                    {
                        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                        circle(bgr_image, centers[i], (int)radius[i], color, 2);
                        rectangle(bgr_image, start_point, end_point, color, 2);

                        bool inside = inside_rect(centers[i].x, centers[i].y, left_x, 0, right_x, frame_height);
                        // cout << "Radius" << radius[i] << "\n";
                        if (inside == 0)
                        {
                            putText(bgr_image, "False", Point(frame_width - 100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);
                            // cout << right_size(depth_estimation(radius[i]))<< "\n";
                            if (right_size(depth_estimation(radius[i])))
                            {
                                putText(bgr_image, "True", Point(frame_width - 100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, LINE_AA);
                            }
                            else
                            {
                                putText(bgr_image, "False", Point(frame_width - 100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);
                            }
                        }
                        else if (inside == 1)
                        {
                            putText(bgr_image, "True", Point(frame_width - 100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, LINE_AA);
                            // cout << right_size(depth_estimation(radius[i])) << "\n";
                            if (right_size(depth_estimation(radius[i])))
                            {
                                putText(bgr_image, "True", Point(frame_width - 100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, LINE_AA);
                            }
                            else
                            {
                                putText(bgr_image, "False", Point(frame_width - 100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);
                            }
                        }
                        double estimated_length = depth_estimation(radius[i]);
                        putText(bgr_image, to_string(estimated_length), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, LINE_AA);
                        // putText(bgr_image, to_string(estimated_length), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, LINE_AA);

                        // //forward:
                        // if((centers[i].x<right_x) && (centers[i].x>left_x) && (estimated_length>20)){
                        //     snprintf(kør lige ud);
                        // }//back:
                        // else if((centers[i].x<right_x) && (centers[i].x>left_x) && (estimated_length<10)){
                        //     snprintf(kør bagud);
                        // }//right
                        string file_type = ".png";
                        if ((centers[i].x > right_x))
                        {

                            printf("Turn right");
                            putText(bgr_image, "Turning right", Point(20, frame_height - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);

                            // bridge->send("vel=0.2, tr=0:turn=10");
                            int line = 0;
                            snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0: turn=-2,time=2");
                            snprintf(lines[line++], MAX_LEN, "vel=0,event=2:dist=1");
                            sendAndActivateSnippet(lines, line);
                            // make sure event 2 is cleared
                            bridge->event->isEventSet(2);

                            // string p1 = "image_" + to_string(pic_id) + ".png";
                            // cout << p1 << endl;
                            // pic_id += 1;
                            // imwrite(p1, bgr_image);
                            state = 10;

                        } // left
                        else if ((centers[i].x < left_x))
                        {

                            printf("Turn left");
                            putText(bgr_image, "Turning left", Point(20, frame_height - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);

                            // bridge->send("vel=0.2, tr=0:turn=-10");
                            int line = 0;
                            snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0: turn=2,time=2");
                            snprintf(lines[line++], MAX_LEN, "vel=0,event=2:dist=1");
                            sendAndActivateSnippet(lines, line);
                            // make sure event 2 is cleared
                            bridge->event->isEventSet(2);
                            // make sure event 2 is cleared
                            // printf("before isevent2\n");
                            // bridge->event->isEventSet(2);
                            // printf("after isevent2\n");

                            // string p1 = "image_" + to_string(pic_id) + ".png";
                            // cout << p1 << endl;
                            // pic_id += 1;
                            // imwrite(p1, bgr_image);
                            state = 10;
                        }
                        else if ((centers[i].x < right_x) && (centers[i].x > left_x))
                        {

                            printf("I am within x and y 8)");
                            putText(bgr_image, "Ball within bounds", Point(20, frame_height - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);

                            // string p1 = "image_" + to_string(pic_id) + ".png";
                            // cout << p1 << endl;
                            // pic_id += 1;
                            // imwrite(p1, bgr_image);
                            if ((inside) && (estimated_length < 30))
                            {
                                state = 999;
                            }
                            else if ((inside) && (estimated_length > 30))
                            {
                                int line = 0;
                                snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.2 :dist=0.05");
                                snprintf(lines[line++], MAX_LEN, "vel=0,event=2:dist=1");
                                sendAndActivateSnippet(lines, line);
                                // make sure event 2 is cleared
                                bridge->event->isEventSet(2);
                            }
                            else
                            {
                                state = 10;
                            }
                        }
                        c_num += 1;
                    }
                }
            }
        }
        else // Det her er rimeligt dårligt lige nu :( Nogle gange fucker den lidt op 
        {
            printf("Searching for tennis ball");
            putText(bgr_image, "Seaching for tennis ball", Point(20, frame_height - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, LINE_AA);

            // bridge->send("vel=0.2, tr=0:turn=10");
            int line = 0;
            snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0: turn=-2,time=2");
            snprintf(lines[line++], MAX_LEN, "vel=0,event=2:dist=1");
            sendAndActivateSnippet(lines, line);
            // make sure event 2 is cleared
            bridge->event->isEventSet(2);
            state = 10;
        }
        video.write(bgr_image);
        break;
    }
    case 999:
    default:
        printf("mission 1 ended \n");
        bridge->send("oled 5 \"mission 1 ended.\"");
        finished = true;
        break;
    }
    return finished;
}

void UMission::openLog()
{
    // make logfile
    const int MDL = 32;
    const int MNL = 128;
    char date[MDL];
    char name[MNL];
    UTime appTime;
    appTime.now();
    appTime.getForFilename(date);
    // construct filename ArUco
    snprintf(name, MNL, "log_mission_%s.txt", date);
    logMission = fopen(name, "w");
    if (logMission != NULL)
    {
        const int MSL = 50;
        char s[MSL];
        fprintf(logMission, "%% Mission log started at %s\n", appTime.getDateTimeAsString(s));
        fprintf(logMission, "%% Start mission %d end mission %d\n", fromMission, toMission);
        fprintf(logMission, "%% 1  Time [sec]\n");
        fprintf(logMission, "%% 2  mission number.\n");
        fprintf(logMission, "%% 3  mission state.\n");
    }
    else
        printf("#UCamera:: Failed to open image logfile\n");
}

void UMission::closeLog()
{
    if (logMission != NULL)
    {
        fclose(logMission);
        logMission = NULL;
    }
}
