#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include <conio.h>
#include <functional> // 헤더파일 포함
#include <memory> // 헤더파일 포함
#include <cstdio> // 헤더파일 포함
#include <iostream> // 헤더파일 포함
#include <string> // 헤더파일 포함
#include "opencv2/opencv.hpp" // 헤더파일 포함
#include "rclcpp/rclcpp.hpp" // 헤더파일 포함
#include "sensor_msgs/msg/image.hpp" // 헤더파일 포함
#include "linetracer_ros2/camera.hpp" // 헤더파일 포함
#include "linetracer_ros2/vision.hpp" // 헤더파일 포함

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

using std::placeholders::_1; // bind 함수 대체 역할, wrapping 하는 함수의 첫 번째 인자

class LinetracerWindow : public rclcpp::Node // 시간 단위 지정을 위한 네임스페이스 사용
{
public:
    LinetracerWindow() : Node("LinetracerWindow") // 생성자 정의 및 초기화
    {
        size_t depth = rmw_qos_profile_default.depth; // QoS profile의 깊이 설정
        rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability; // Qos profile의 신뢰성 설정 값 설정
        rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; // QoS profile의 히스토리 설정 값 설정
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth)); // QoS profile 생성 및 위의 설정 값들로 초기화
        reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; // Qos profile의 신뢰성 값을 BEST_EFFORT로 설정
        qos_profile.reliability(reliability_policy); // Qos profile 신뢰성 설정

        pi_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("pi_image", qos_profile, // iamge 토픽을 구독하는 camera_subscriber 생성
                                    std::bind(&LinetracerWindow::show_pi_image, this, _1)); // show_image를 콜백 함수로 사용, 앞에서 설정한 _1 값으로 메제지 전달
        cv::namedWindow("Pi_CAM", cv::WINDOW_AUTOSIZE); // 이미지를 띄울 윈도우 생성

        auto qos_profile_dxl = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS profile 생성 및 초기화
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile_dxl);
        // 선속도와 각속도를 보낼 퍼블리셔 생성
        timer_ = this->create_wall_timer(100ms, std::bind(&LinetracerWindow::publish_velcmd_msg, this));
        // 일정 시간마다 이미지 발행을 위한 타이머 생성
    }

private:
    void publish_velcmd_msg() // 콜백 함수
    {
        auto static msg = geometry_msgs::msg::Twist(); 
        
        if (_kbhit()) // 키보드 입력시
        {        
            char c = _getch(); // 입력 값을 저장
            switch(c) // 조건문
            {
            case 's': // 입력 값이 s인 경우
                msg.linear.x  = 0; // 선속도 
                msg.angular.z = 0; // 각속도
                break; // 조건문 탈출
            default: // 위의 경우가 아닌 경우
                msg.linear.x = msg.linear.x; // 선속도 
                msg.angular.z = msg.angular.z; // 각속도   
                break; // 조건문 탈출
            }
            RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf", msg.linear.x, msg.angular.z); // 정보 출력
            dynamixel_publisher_->publish(msg); // 메세지 퍼블리셔 
        }   
        msg.linear.x = 0.3; // 선속도
        msg.angular.z = error / 70; // 각속도  
        RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf", msg.linear.x, msg.angular.z); // 정보 출력
        dynamixel_publisher_->publish(msg); // 메세지 퍼블리셔
    }
    void show_pi_image(const sensor_msgs::msg::Image::SharedPtr msg) const // 콜백 함수
    {
        // RCLCPP_INFO(this->get_logger(), "Received pi_image #%s", msg->header.frame_id.c_str()); // info 메세지 출력 
        
        // Convert to an OpenCV matrix by assigning the data.
        Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), // msg 데이터 값으로 mat 객체 생성
                const_cast<unsigned char*>(msg->data.data()), msg->step);
        if (msg->encoding == "rgb8") { // 인코딩 형식이 rgb8인 경우
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR); // RGB형식을  BGR형식으로 변환
        }

        Mat cvframe = frame; // 변환시킨 mat 객체 저장
        Mat RP; // 관심 영역 mat 객체
        Mat RP_Color; // 관심 영역 컬러 영상 mat 객체
        Mat labels; // mat 객체 선언
        Mat stats; // mat 객체 선언 
        Mat centroid; // mat 객체 선언
        int cnt = 0; // 횟수를 저장할 정수형 변수

        RP = get_ROI(frame); // 관심 영역 추출 및 이진화
        cnt = connectedComponentsWithStats(RP, labels, stats, centroid); // 객체 검출
        draw_Rect(&RP_Color, RP, cnt, stats, centroid, &ps_center); // 검출된 객체 점과 사각형으로 표시
        get_error(frame, cnt, stats, centroid, ps_center, &error); // 중심점과 객체간의 거리 측정
        cout << "error: " << error << endl; // 결과 메세지 출력
        //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
        //cv::imshow("PI_CAM", RP_Color); ; // 결과 영상 출력
        imshow("Pi_CAM", RP_Color); // 결과 영상 출력
        waitKey(1); // 키보드 값 입력 대기
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pi_camera_subscriber_; // ensor_msgs::msg::Image 타입 변수 선언
    rclcpp::TimerBase::SharedPtr timer_; // 타이머 변수 선언
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_; // geometry_msgs::msg::Twist 타입 변수 선언
    mutable double error = 0; // 에러를 저장할 실수형 변수
    mutable Mat ps_center; // 과거 중심점을 저장할 변수 
    //size_t count_;
};

int main(int argc, char* argv[]) // 메인 함수
{
    rclcpp::init(argc, argv); // ROS 2를 argc와 argv 값으로 초기화
    auto node = std::make_shared<LinetracerWindow>(); // LinetracerLinux 클래스의 객체인 node를 생성
    std::cout << "LineTracer:" << std::endl; // 안내 메세지 출력
    rclcpp::spin(node); // node를 스핀시켜 지정된 콜백함수 실행
    rclcpp::shutdown(); // 종료 `Ctrl + c`와 같은 인터럽트 시그널 예외 상황에서는 rclcpp::shutdown 함수로 노드를 소멸하고 프로세스를 종료
    return 0; // 함수 종료

}

