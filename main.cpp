/*
 * main.cpp
 *
 *  Created on: 2016. 8. 3.
 *      Author: Bryan Wodi <talk2kamp@gmail.com>
 *          ADAPTED from robotis tutorial {ball_following}
 *  Purpose: sprint towards an object
 */
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME        "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0; 
static int iHighS = 255;

static int iLowV = 0;
static int iHighV = 255;


void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

void setRangeParams()
{
    std::cout <<"Setting the range for thresholding" <<std::endl;
    std::cout <<"Press the ENTER key when finished!\n"<<std::endl;

    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    //values for inRange thresholding of red colored objects
    cv::namedWindow("Colour Control", CV_WINDOW_AUTOSIZE);

    //Create trackbars in "Colour Control" window
    cvCreateTrackbar("LowH", "Colour Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Colour Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Colour Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Colour Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Colour Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Colour Control", &iHighV, 255);
    
    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
    
        cv::Mat curr_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);
        cv::Mat curr_hsv, curr_thresholded;
        if( curr_frame.data )
        {        
            //first convert cam image to bgr before hsv
            cv::cvtColor(curr_frame,curr_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(curr_frame,curr_hsv,cv::COLOR_BGR2HSV);
            
            cv::inRange(curr_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),curr_thresholded);
            cv::GaussianBlur(curr_thresholded,curr_thresholded,cv::Size(9,9),2,2);
            //morphological opening (remove small objects from the foreground)
            cv::erode(curr_thresholded, curr_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
            cv::dilate( curr_thresholded, curr_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

            //morphological closing (fill small holes in the foreground)
            cv::dilate( curr_thresholded, curr_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
            cv::erode(curr_thresholded, curr_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        }
        cv::imshow("Thresholded Image",curr_thresholded);
        cv::imshow("Original Image",curr_frame);
        
        if(cv::waitKey(30) == 10) 
        {
            cv::destroyAllWindows();
            break;
        }
    }
    
}
int main(void)
{
	signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    follower.DEBUG_PRINT = true;

    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Failed to initialize Motion Manager!\n");
        return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;

    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;

        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
    }
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);

    

    //values for inRange thresholding of red colored objects
    setRangeParams();
    std::cout << std::endl;
    std::cout << "iLowH: " <<iLowH <<std::endl;
    std::cout << "iHighH: " <<iHighH <<std::endl;
    std::cout << "iLowS: " <<iLowS <<std::endl;
    std::cout << "iHighS: " <<iHighS <<std::endl;
    std::cout << "iLowV: " <<iLowV <<std::endl;
    std::cout << "iHighV: " <<iHighV <<std::endl;
    std::cout << "All set!\n" <<std::endl;

    printf("Press the ENTER key to begin!\n");
    getchar();
   
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    //values for reporting the X and Y vals for found circle
    int iLastX = -1; 
    int iLastY = -1;

    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        
        cv::Mat mat_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);

        if( mat_frame.data )
        {
            cv::Mat img_hsv, img_thresholded;
            //first convert cam image to BGR to properly correctly convert to HSV
            cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
            
            cv::inRange(img_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),img_thresholded);
            cv::GaussianBlur(img_thresholded,img_thresholded,cv::Size(9,9),2,2);
            
            //morphological opening (remove small objects from the foreground)
            //cv::erode(img_thresholded, img_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
            //cv::dilate( img_thresholded, img_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

            //morphological closing (fill small holes in the foreground)
            //cv::dilate( img_thresholded, img_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
            //cv::erode(img_thresholded, img_thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

            //Calculate the moments of the thresholded image
            cv::Moments oMoments = cv::moments(img_thresholded);

            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;



            // if the area <= 100,000 then there are no objects 
            //in the image and it's all just noise 
            if (dArea > 100000)
            {
                //calculate the position of the ball
                int posX = (int)(dM10 / dArea);
                int posY = (int)(dM01 / dArea);
                iLastX = posX;
                iLastY = posY;

                //find circle using HoughCircles
	            std::vector<cv::Vec3f> circles;
            	cv::HoughCircles(img_thresholded, circles, CV_HOUGH_GRADIENT,1,img_thresholded.rows/4, 200, 20);

	            if (circles.size() > 0)
	            {
	            	for (size_t i = 0; i < circles.size(); ++i)
		            {
		            	cv::Point centre( cvRound(circles[i][0]), cvRound(circles[i][1]) );
		            	int radius = cvRound(circles[i][2]);
		            	
		            	if (radius >= 10)
		            	{
		            		Point2D new_ball_pos(iLastX,iLastY);
			                tracker.Process(new_ball_pos);
				            follower.Process(tracker.ball_position);
		            	}
		            }
	            }
	            else
	            	std::cout << " currently not detecting any circles" << std::endl;
	            
                 cv::imshow("Thresholded Image",img_thresholded);
				 if(cv::waitKey(30) == 27) break;
            }//inner if
        } //outer if
    } //while
    return 0;
}
