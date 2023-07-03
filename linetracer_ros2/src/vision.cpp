#include "opencv2/opencv.hpp" // 헤더파일 포함 
#include <iostream> // 헤더파일 포함
#include "linetracer_ros2/vision.hpp" // 헤더파일 포함

using namespace std; // 객체명 앞에 std:: 생략
using namespace cv; // 객체명 앞에 cv:: 생략

Mat get_ROI(Mat frame) 
{
    Mat RP; 
    Mat gray; 
    Mat th; 

    RP = frame(Rect(0, 200, 320, 40)); 
    cvtColor(RP, gray, COLOR_BGR2GRAY); 
    Scalar g_avr = mean(gray); 
    gray = gray + (150 - g_avr[0]); 
    threshold(gray, th, 220, 255, THRESH_BINARY); 
    return th; 
}

void draw_Rect(Mat* color, Mat frame, int cnt, Mat stats, Mat centroid, Mat* ps_center) 
{
    cvtColor(frame, *color, COLOR_GRAY2BGR); 
    if(cnt == 1) 
    {
       circle(*color, Point(ps_center->at<double>(1,0), ps_center->at<double>(1,1)), 1, Scalar(0,0,255), 5); 
    }
    else if(cnt > 1) 
    {
        for(int i = 0; i < cnt; i++) 
        {
            int* label = stats.ptr<int>(i); 
            if((label[4] < 80) || (label[4] > 2000)) continue; 
            rectangle(*color, Rect(label[0], label[1], label[2], label[3]), Scalar(0,0,255)); 
            circle(*color, Point(centroid.at<double>(i,0), centroid.at<double>(i,1)), 1 ,Scalar(0,0,255), 5); 
        }
        *ps_center = centroid; 
    }
    
}

void get_error(Mat frame, int cnt, Mat stats, Mat centroid, Mat ps_center, double* error_x) 
{
    int win_x = frame.cols / 2; 
    double line_x; 
    if(cnt == 1)
    {
        line_x = ps_center.at<double>(1,0); 
        *error_x = win_x - line_x; 
    }
    else if(cnt > 1) 
    {
        for(int i = 0; i < cnt; i++) 
        {
            int* label = stats.ptr<int>(i);
            if((label[4] < 100) || (label[4] > 500)) continue; 
            line_x = centroid.at<double>(i,0); 
            *error_x = win_x - line_x; 
        }
    }
}
