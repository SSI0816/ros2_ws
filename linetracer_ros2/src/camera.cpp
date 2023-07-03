#include <cstdio> // 헤더파일 포함
#include <iostream> // 헤더파일 포함
#include <memory> // 헤더파일 포함
#include <string> // 헤더파일 포함
#include <utility> // 헤더파일 포함
#include <sstream> // 헤더파일 포함
#include "opencv2/opencv.hpp" // 헤더파일 포함
#include "rclcpp/rclcpp.hpp" // 헤더파일 포함 
#include "sensor_msgs/msg/image.hpp" // 헤더파일 포함
#include "linetracer_ros2/camera.hpp" // 헤더파일 포함

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string mat_type2encoding(int mat_type) // 인코딩 타입인 mat_type이 전달되며 해당 타입을 기반으로 문자열 형태의 인코딩을 결정
{
    switch (mat_type) { // 조건문
    case CV_8UC1: // mat_type이 8비트 unsigned char(흑백)인 경우
        return "mono8"; // mono8을 mat_type으로 리턴
    case CV_8UC3: // mat_type이 8비트 unsigned char(BGR)인 경우
        return "bgr8"; // bgr8을 mat_type으로 리턴
    case CV_16SC1: // mat_type이 16비트 unsigned char(흑백)인 경우
        return "mono16"; // mono16을 mat_type으로 리턴
    case CV_8UC4: // mat_type이 8비트 unsigned char(BGRA)인 경우
        return "rgba8"; // rgba8을 mat_type으로 리턴
    default: // 위의 경우에 해당되지 않는 경우
        throw std::runtime_error("Unsupported encoding type"); // 지원되지 않는 인코딩 타입 메세지 출력
    }
}


/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
//void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg)
void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg) // 이미지를 메세지로 변환
{
    // copy cv information into ros message
    msg.height = frame.rows; // 영상의 rows값 저장
    msg.width = frame.cols; // 영상의 cols값 저장
    msg.encoding = mat_type2encoding(frame.type()); // 영상의 인코딩 타입 결정
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step); // 영상의 step 값을 msg의 멤버변수 step에 저장
    size_t size = frame.step * frame.rows; // 영상의 크기 계산 
    msg.data.resize(size); // msg.data 값을 영상에 맞게 재조정
    memcpy(&msg.data[0], frame.data, size); // fram.data에서 msg.data로 데이터 복사
    //msg.header.frame_id = std::to_string(frame_id);
    msg.header.frame_id = "camera"; // msg의 frame id 설정
}

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int encoding2mat_type(const std::string& encoding) // sensor_msgs::Image의 인코딩 타입을 OpenCV의 이미지 인코딩 타입으로 변환하는 역할
{
    if (encoding == "mono8") { // 인코딩 타입이 mono8인 경우
        return CV_8UC1; // 8비트 unsigned char(흑백)으로 리턴
    }
    else if (encoding == "bgr8") { // 인코딩 타입이 bgr8인 경우
        return CV_8UC3; // 8비트 unsigned char(BGR)로 리턴
    }
    else if (encoding == "mono16") { // 인코딩 타입이 mono16인 경우
        return CV_16SC1; // 16비트 signed char(흑백)
    }
    else if (encoding == "rgba8") { // 인코딩 타입이 rgba8인 경우
        return CV_8UC4; // 8비트 unsigned cha(BGRA)
    }
    else if (encoding == "bgra8") { // 인코딩 타입이 bgra8인 경우
        return CV_8UC4; // 8비트 unsigned cha(BGRA)
    }
    else if (encoding == "32FC1") { // 인코딩 타입이 32FC1인 경우
        return CV_32FC1; // 32비트 floating-point(RGB)
    }
    else if (encoding == "rgb8") { // 인코딩 타입이 rgb8인 경우
        return CV_8UC3; // 8비트 unsigned char(RGB)
    }
    else { // 위의 경우들이 아닌 경우
        throw std::runtime_error("Unsupported encoding type"); // 지원되지 않는 인코딩 타입 메세지 출력
    }
}

