// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional> // 헤더파일 포함
#include <memory> // 헤더파일 포함
#include <cstdio> // 헤더파일 포함
#include <iostream> // 헤더파일 포함
#include <string> // 헤더파일 포함
#include "opencv2/opencv.hpp" // 헤더파일 포함
#include "rclcpp/rclcpp.hpp" // 헤더파일 포함
#include "sensor_msgs/msg/image.hpp" // 헤더파일 포함
#include "camera_ros2/camera.hpp" // 헤더파일 포함

using std::placeholders::_1; // bind 함수 대체 역할, wrapping 하는 함수의 첫 번째 인자

class CameraSubscriber : public rclcpp::Node //  rclcpp::Node를 상속하는 CameraSubscriber 클래스 정의
{
public:
  CameraSubscriber() : Node("Camera_subscriber") // rclcpp::Node를 상속하는 CameraSubscriber 클래스의 생성자를 Camera_subscriber로 생성
  {
    size_t depth = rmw_qos_profile_default.depth; // QoS profile의 깊이 설정
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability; // Qos profile의 신뢰성 설정 값 설정
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; // QoS profile의 히스토리 설정 값 설정
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth)); // QoS profile 생성 및 위의 설정 값들로 초기화
    reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; // Qos profile의 신뢰성 값을 BEST_EFFORT로 설정
    qos_profile.reliability(reliability_policy); // Qos profile 신뢰성 설정

    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image", qos_profile, // iamge 토픽을 구독하는 camera_subscriber 생성
                                    std::bind(&CameraSubscriber::show_image, this, _1)); // show_image를 콜백 함수로 사용, 앞에서 설정한 _1 값으로 메제지 전달
    cv::namedWindow("USB_CAM", cv::WINDOW_AUTOSIZE); // 이미지를 띄울 윈도우 생성

    pi_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("pi_image", qos_profile, // iamge 토픽을 구독하는 camera_subscriber 생성
                                    std::bind(&CameraSubscriber::show_pi_image, this, _1)); // show_image를 콜백 함수로 사용, 앞에서 설정한 _1 값으로 메제지 전달
    cv::namedWindow("Pi_CAM", cv::WINDOW_AUTOSIZE); // 이미지를 띄울 윈도우 생성
  }

private:
  void show_image(const sensor_msgs::msg::Image::SharedPtr msg) const // 콜백 함수
  {
    RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str()); // info 메세지 출력 
    
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), // msg 데이터 값으로 mat 객체 생성
            const_cast<unsigned char*>(msg->data.data()), msg->step);
    if (msg->encoding == "rgb8") { // 인코딩 형식이 rgb8인 경우
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR); // RGB형식을  BGR형식으로 변환
    }

    cv::Mat cvframe = frame; // 변환시킨 mat 객체 저장
    //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
    cv::imshow("USB_CAM", cvframe); ; // 결과 영상 출력
    cv::waitKey(1); // 키보드 값 입력 대기
  }
  void show_pi_image(const sensor_msgs::msg::Image::SharedPtr msg) const // 콜백 함수
  {
    RCLCPP_INFO(this->get_logger(), "Received pi_image #%s", msg->header.frame_id.c_str()); // info 메세지 출력 
    
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), // msg 데이터 값으로 mat 객체 생성
            const_cast<unsigned char*>(msg->data.data()), msg->step);
    if (msg->encoding == "rgb8") { // 인코딩 형식이 rgb8인 경우
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR); // RGB형식을  BGR형식으로 변환
    }

    cv::Mat cvframe = frame; // 변환시킨 mat 객체 저장
    //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
    cv::imshow("PI_CAM", cvframe); ; // 결과 영상 출력
    cv::waitKey(1); // 키보드 값 입력 대기
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_; // ensor_msgs::msg::Image 타입 변수 선언
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pi_camera_subscriber_; // ensor_msgs::msg::Image 타입 변수 선언
};

int main(int argc, char * argv[]) // 메인 함수 
{
  rclcpp::init(argc, argv); // ROS 2를 argc와 argv 값으로 초기화
  auto node = std::make_shared<CameraSubscriber>(); // CameraSubscriber 클래스의 객체인 node를 생성
  rclcpp::spin(node); // node를 스핀시켜 지정된 콜백함수 실행
  rclcpp::shutdown(); // 종료 `Ctrl + c`와 같은 인터럽트 시그널 예외 상황에서는 rclcpp::shutdown 함수로 노드를 소멸하고 프로세스를 종료
  return 0; // 함수 종료
}