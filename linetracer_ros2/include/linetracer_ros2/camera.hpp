#ifndef CAMERA_HPP_ // CAMERA_HPP가 정의 되어 있지 않은 경우
#define CAMERA_HPP_ // CAMERA_NPP를 정의

#include "opencv2/opencv.hpp" // 헤더파일 포함
#include "sensor_msgs/msg/image.hpp" // 헤더파일 포함
#include "rclcpp/rclcpp.hpp" // 헤더파일 포함

std::string mat_type2encoding(int mat_type); // 인코딩 타입인 mat_type이 전달되며 해당 타입을 기반으로 문자열 형태의 인코딩을 결정하는 함수
//void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg);
void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg); // 이미지를 메세지로 변환
int encoding2mat_type(const std::string& encoding); // sensor_msgs::Image의 인코딩 타입을 OpenCV의 이미지 인코딩 타입으로 변환하는 역할

#endif  // CAMERA_HPP_ // 중복 포함 방지 지시문 닫기


