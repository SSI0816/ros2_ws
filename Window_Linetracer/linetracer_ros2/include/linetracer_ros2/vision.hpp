#ifndef _VISION_HPP_ // VISION_HPP가 정의 되어 있지 않은 경우
#define _VISION_HPP_ // VISION_HPP를 정의
#include "opencv2/opencv.hpp" // 헤더파일 포함
using namespace cv; // 객체명 앞에 cv:: 생략

Mat get_ROI(Mat frame); // 관심영역 추출 및 이진화 함수
void draw_Rect(Mat* color, Mat frame, int cnt, Mat stats, Mat centroid, Mat* ps_center); // 검출된 객체 원과 사각형으로 표시하여 그리는 함수
void get_error(Mat frame, int cnt, Mat stats, Mat centroid, Mat ps_center, double* error_x); // 중심점과 객체간의 거리를 계산하여 오차를 구하는 함수
#endif // 중복 포함 방지 지시문 닫기