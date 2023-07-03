#include <chrono> // 헤더파일 포함
#include <functional> // 헤더파일 포함
#include <memory> // 헤더파일 포함
#include <string> // 헤더파일 포함
#include <cstdio> // 헤더파일 포함
#include <iostream> // 헤더파일 포함
#include <utility> // 헤더파일 포함
#include "opencv2/opencv.hpp" // 헤더파일 포함
#include "rclcpp/rclcpp.hpp" // 헤더파일 포함
#include "sensor_msgs/msg/image.hpp" // 헤더파일 포함
#include "linetracer_ros2/camera.hpp" // 헤더파일 포함
#include "geometry_msgs/msg/twist.hpp" // 헤더파일 포함
#include "linetracer_ros2/dxl.hpp" // 헤더파일 포함

using namespace std; // 객체명 앞에 std:: 생략
using namespace cv; // 객체명 앞에 cv:: 생략

using std::placeholders::_1; // bind 함수 대체 역할, wrapping 하는 함수의 첫 번째 인자

using namespace std::chrono_literals; // 시간 단위 지정을 위한 네임스페이스 사용

class LinetracerLinux : public rclcpp::Node // rclcpp::Node를 상속하는 LinetracerLinux 클래스 정의
{
public:
  LinetracerLinux() : Node("linetracer_linux"), pi_count_(1), width_(640), height_(480)// 생성자 정의 및 초기화
  {
    size_t depth = rmw_qos_profile_default.depth; // QoS profile의 깊이 설정
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; // QoS profile의 신뢰성 BEST_EFFORT로설정
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; // QoS profile의 히스토리 설정 값 설정
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));// QoS profile 생성 및 위의 설정 값들로 초기화
    qos_profile.reliability(reliability_policy); // 신뢰성 설정 값 설정
   auto qos_profile_dxl = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS profile 생성 및 초기화
    dynamixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_profile_dxl, std::bind(&LinetracerLinux::subscribe_topic_message, this, _1)); // 선속도와 각속도를 받을 서브스크라이버 생성
        
    if(!dxl.dxl_open()) RCLCPP_ERROR(this->get_logger(), "dynamixel open error");; // dxl 에러 처리

    pi_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("pi_image", qos_profile); // 이미지 메시지를 발행하기 위한 퍼블리셔 생성
    pi_timer_ = this->create_wall_timer(30ms, std::bind(&LinetracerLinux::publish_pi_image, this)); // 일정 시간마다 이미지 발행을 위한 타이머 생성

    // pi camera
    pi_cap_.open(src_, cv::CAP_GSTREAMER); // GStreamer를 사용하여 카메라 영상을 열기
    
    if (!pi_cap_.isOpened()) { // 비디오 열기 실패시 
        RCLCPP_ERROR(this->get_logger(), "Could not open pi video stream"); // 비디오 열기에 실패시 에러 메세지 출력          
    }

  }
  ~LinetracerLinux() // 소멸자
  {
    dxl.dxl_close(); // dxl 닫기
  }
private:
  void publish_pi_image() // 콜백 함수
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();  // 보낼 메시지 타입 설정
    msg->is_bigendian = false; 
    // is_bigendian이 false로 설정되면 이미지 데이터는 리틀 엔디안 방식(데이터의 가장 낮은 자릿수부터 메모리에 저장되는 방식)으로 저장되고 해석
    pi_cap_ >> pi_frame_; // 카메라 영상 frame으로 받기
    if (!pi_frame_.empty()) { // 카메라 영상이 비어있지 않은 경우
        // Convert to a ROS image
        // cv::flip(frame, frame, 1); // Flip the frame if needed
        //convert_frame_to_message(frame_, count_, *msg);
        convert_frame_to_message(pi_frame_, *msg); // 영상을 메세지로 변환
        
        // Publish the image message and increment the frame_id.
        // RCLCPP_INFO(this->get_logger(), "Publishing Pi_image #%zd", pi_count_++); // info 정보를 출력하고 카운트 값 증가
        pi_camera_publisher_->publish(std::move(msg)); // 메세지 퍼블리셔      

        
    }
    else {
        RCLCPP_INFO(this->get_logger(), "pi_frame empty"); // 카메라 영상이 없는 경우 에러 메세지 출력  
    }
  }
  void subscribe_topic_message(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Received message: %lf,%lf", msg->linear.x,msg->angular.z); // 정보 출력
        int speed_L = 0, speed_R = 0; // 좌우 바퀴 속도를 저장할 변수 선언
        dxl.dxl_get_LR(msg->linear.x, msg->angular.z, &speed_L, &speed_R); // 선속도, 각속도를 좌우바퀴 속도로 변환
        bool result = dxl.dxl_set_velocity(speed_L, -speed_R); // 모터 제어 함수
        cout << "speed_L: " << speed_L / 4.37 << "   " << "speed_R: " << speed_R / 4.37 << endl; // 결과 메세지 출력
        if(result == false) // 에러 발생시
        {
           RCLCPP_ERROR(this->get_logger(), "dxl_set_velocity error"); // 에러 메세지 출력 
        }         
    }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dynamixel_subscriber_; // 각속도, 선속도를 받을 서브크스라이버 선언
  Dxl dxl; // dxl 객체 생성
  size_t count_; // id 값 변수 선언
  rclcpp::TimerBase::SharedPtr pi_timer_; // 타이머 변수 선언
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pi_camera_publisher_; // 이미지 발생 퍼블리셔 선언
  size_t pi_count_; // 이미지 프레임 id 값 변수 선언
  cv::VideoCapture pi_cap_; // 이미지 캡처 변수 선언
  std::string src_ = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)320, height=(int)240, format=(string)NV12, framerate=(fraction)30/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)320, height=(int)240, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  // 비디오 소스 설정 값 설정
  cv::Mat pi_frame_; // 영상을 받을 mat 객체 선언
  size_t width_; // 프레임의 너비 값 변수 선언
  size_t height_; // 프레임의 놓이 값 변수 선언
  
};

int main(int argc, char * argv[]) // 메인 함수
{
  rclcpp::init(argc, argv); // ROS 2를 argc와 argv 값으로 초기화
  auto node = std::make_shared<LinetracerLinux>(); // LinetracerLinux 클래스의 객체인 node를 생성
  rclcpp::spin(node); // node를 스핀시켜 지정된 콜백함수 실행
  rclcpp::shutdown(); // 종료 `Ctrl + c`와 같은 인터럽트 시그널 예외 상황에서는 rclcpp::shutdown 함수로 노드를 소멸하고 프로세스를 종료
  return 0; // 함수 종료
}