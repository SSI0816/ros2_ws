cmake_minimum_required(VERSION 3.5) // 운영체제에 설치된 cmake 최소 요구 버전(3.5 버전 이상
project(camera_ros2) // 패키지 이름 기재, package.xml에서 입력한 이름 그래도 사용

# Default to C99
if(NOT CMAKE_C_STANDARD) // c언어 기준이 되는 버전 없을시 
  set(CMAKE_C_STANDARD 99) // c언어 기준이 되는 버전 99로 설정
endif() 

# Default to C++14
if(NOT CMAKE_CXX_STANDARD) // c++ 기준이 되는 버전 없을시 
  set(CMAKE_CXX_STANDARD 14) // c++ 기준이 되는 버전 14로 설정
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") GNUCXX 컴파일러이거나 CXX 컴파일러 ID가 "Clang"인 경우
  add_compile_options(-Wall -Wextra -Wpedantic) // Wall, -Wextra, -Wpedantic을 추가
endif()

# find dependencies
find_package(ament_cmake REQUIRED) // clcpp 사용시 ament_cmake, rclcpp 기본 포함
find_package(rclcpp REQUIRED) // 해당 패키지의 헤더파일 경로, 라이브러리 파일등의 정보를 자동으로 찾아줌
find_package(std_msgs REQUIRED) // 해당 패키지의 헤더파일 경로, 라이브러리 파일등의 정보를 자동으로 찾아줌
find_package(sensor_msgs REQUIRED) // 해당 패키지의 헤더파일 경로, 라이브러리 파일등의 정보를 자동으로 찾아줌
find_package(OpenCV REQUIRED) // 해당 패키지의 헤더파일 경로, 라이브러리 파일등의 정보를 자동으로 찾아줌

include_directories(include) // include 디렉토리에 있는 헤더파일 포함

add_executable(camera_publisher src/camera_publisher.cpp src/camera.cpp)
// 실행파일명과 소스파일명 지정
ament_target_dependencies(camera_publisher rclcpp sensor_msgs std_msgs OpenCV)
// 실행파일이 의존하는 패키지들을 명시
add_executable(camera_subscriber src/camera_subscriber.cpp src/camera.cpp)
// 실행파일명과 소스파일명 지정
ament_target_dependencies(camera_subscriber rclcpp sensor_msgs std_msgs OpenCV)
// 실행파일이 의존하는 패키지들을 명시
install(TARGETS
  camera_publisher
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME}) // 생성된 실행 파일들을 설치할 위치를 지정
  // 설치 경로는 "lib/${PROJECT_NAME}"로 지정되며, 실행 파일들이 해당 위치에 설치

if(BUILD_TESTING) // BUILD_TESTING의 변수가 true인 경우에만 실행
  find_package(ament_lint_auto REQUIRED) ament_lint_auto 패키지를 찾고 의존성을 확인하기 위해 사용
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies() // 테스트에 필요한 의존성 패키지들을 찾고 설정
endif()

ament_package() // 현재 패키지의 정보를 Ament에 전달하는 역할

