#include "opencv2/opencv.hpp" // 헤더파일 포함 
#include <iostream> // 헤더파일 포함
#include "linetracer_ros2/vision.hpp" // 헤더파일 포함

using namespace std; // 객체명 앞에 std:: 생략
using namespace cv; // 객체명 앞에 cv:: 생략

Mat get_ROI(Mat frame) // 관심영역 추출 및 이진화 함수
{
    Mat RP; // 관심영역 mat 객체
    Mat gray; // 흑백영상 mat 객체
    Mat th; // 이진화영상 mat 객체

    RP = frame(Rect(0, 200, 320, 40)); // 관심영역 지정
    cvtColor(RP, gray, COLOR_BGR2GRAY); // 컬러영상 흑백으로 전환
    Scalar g_avr = mean(gray); // 영상 평균 밝기 계산
    gray = gray + (150 - g_avr[0]); // 영상 밝기 평균 맞추기
    threshold(gray, th, 220, 255, THRESH_BINARY); // 영상 이진화
    return th; // 이진화된 영상으로 리턴
}

void draw_Rect(Mat* color, Mat frame, int cnt, Mat stats, Mat centroid, Mat* ps_center) // 검출된 객체 원과 사각형으로 표시하여 그리는 함수
{
    cvtColor(frame, *color, COLOR_GRAY2BGR); // 흑백영상 컬러로 전환
    if(cnt == 1) 
    {
       circle(*color, Point(ps_center->at<double>(1,0), ps_center->at<double>(1,1)), 1, Scalar(0,0,255), 5); // 객체 중심에 원 그리기
    }
    else if(cnt > 1) 
    {
        for(int i = 0; i < cnt; i++) // 반복문
        {
            int* label = stats.ptr<int>(i); 
            if((label[4] < 80) || (label[4] > 2000)) continue; // 일정 범위를 넘으면 건너뛰기
            rectangle(*color, Rect(label[0], label[1], label[2], label[3]), Scalar(0,0,255)); // 검출된 객체 바운딩박스 그리기
            circle(*color, Point(centroid.at<double>(i,0), centroid.at<double>(i,1)), 1 ,Scalar(0,0,255), 5); // 검출된 객체 중심에 원 그리기
        }
        *ps_center = centroid; // 현재값을 과거값에 저장
    }
    
}

void get_error(Mat frame, int cnt, Mat stats, Mat centroid, Mat ps_center, double* error_x) // 중심점과 객체간의 거리를 계산하여 오차를 구하는 함수
{
    int win_x = frame.cols / 2; // 영상의 x중심 좌표
    double line_x; 
    if(cnt == 1)
    {
        line_x = ps_center.at<double>(1,0); // 검출된 객체의 중심 x좌표
        *error_x = win_x - line_x; // 오차 값 계산
    }
    else if(cnt > 1) 
    {
        for(int i = 0; i < cnt; i++) // 반복문
        {
            int* label = stats.ptr<int>(i);
            if((label[4] < 100) || (label[4] > 500)) continue; // 객체가 범위 넘어가면 건너뛰기
            line_x = centroid.at<double>(i,0); // 검출된 객체의 중심 x좌표
            *error_x = win_x - line_x; // 오차 값 계산
        }
    }
}
