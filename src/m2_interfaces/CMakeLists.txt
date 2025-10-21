cmake_minimum_required(VERSION 3.8)
project(m2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChannelTwist.msg"
  "msg/JoyData.msg"
  "msg/JoyRgbTime.msg"
  "msg/JoyRumble.msg"
  "msg/JoyTpad.msg"
  "msg/AdcBoardFeedback.msg"
  "msg/AdcPacket.msg"
  "msg/EncoderImuStamped.msg"
  "msg/ImuAngularVel.msg"
  "msg/ImuQuat.msg"
  "msg/HBAUartData.msg"
  "msg/HBAOledButtons.msg"
  "msg/HBAOledDrawText.msg"
  "msg/IOBoardSetPos.msg"
  "srv/JoySetRgb.srv"
  "srv/SetAxisValue.srv"
  "srv/SetPoseValue.srv"
  "srv/SetFloat32.srv"
  "srv/SetInt32.srv"
  "srv/SetPose.srv"
  "srv/SetString.srv"
  "srv/GetBool.srv"
  "srv/VescSetSinTrajParams.srv"
  "srv/AlignChassis.srv"
  "srv/SetPid.srv"
  "action/ActiveCasterAlign.action"
  "action/ActiveCasterYawSinTraj.action"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
