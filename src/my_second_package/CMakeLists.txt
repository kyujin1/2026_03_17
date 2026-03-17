cmake_minimum_required(VERSION 3.8)
project(my_second_package)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필요한 패키지들 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_first_package_msgs REQUIRED) # 액션 메시지가 들어있는 패키지

# 2. 실행 파일 빌드 설정
add_executable(dist_turtle_action_server src/dist_turtle_action_server.cpp)

# 3. 의존성 연결
# C++ 코드 내에서 #include 한 모든 ROS 메시지/기능 패키지를 적어야 합니다.
ament_target_dependencies(dist_turtle_action_server 
  rclcpp 
  rclcpp_action
  turtlesim 
  geometry_msgs
  my_first_package_msgs
)

# 4. 실행 파일을 install 폴더로 설치 (ros2 run 명령어를 위해 필수)
install(TARGETS
  dist_turtle_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# 5. ament_package()는 무조건 맨 마지막에!
ament_package()