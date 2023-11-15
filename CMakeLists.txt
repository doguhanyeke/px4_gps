cmake_minimum_required(VERSION 3.8)
project(px4_gps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Find the gz_Transport library
find_package(gz-transport12
	#REQUIRED COMPONENTS core
	NAMES
		#ignition-transport11 # IGN (Fortress and earlier) no longer supported
		gz-transport12
	#QUIET
)
find_package(gz-msgs9 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sensors7 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(gz-rendering7 REQUIRED)

if(gz-transport12_FOUND)
	add_compile_options(-frtti -fexceptions)

	set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

	if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
		set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
	endif()
endif()

add_executable(px4_gps_sim_pub src/px4_gps_sim_pub.cpp)
add_executable(px4_gps_real_pub src/px4_gps_real_pub.cpp)
add_executable(mavros_gps_real_pub src/mavros_gps_real_pub.cpp)

ament_target_dependencies(px4_gps_sim_pub rclcpp ${Eigen_LIBRARIES}
                        gz-transport12 gz-sensors7 gz-rendering7 gz-msgs9 gz-math7 px4_msgs std_msgs geometry_msgs)
target_include_directories(px4_gps_sim_pub PUBLIC ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(px4_gps_real_pub rclcpp
                        gz-transport12 gz-sensors7 gz-rendering7 gz-msgs9 gz-math7 px4_msgs std_msgs)     
target_include_directories(px4_gps_real_pub PUBLIC ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(mavros_gps_real_pub rclcpp
                        gz-transport12 gz-sensors7 gz-rendering7 gz-msgs9 gz-math7 px4_msgs std_msgs mavros_msgs)                   
target_include_directories(mavros_gps_real_pub PUBLIC ${Eigen3_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(TARGETS
px4_gps_sim_pub
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
px4_gps_real_pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
mavros_gps_real_pub
    DESTINATION lib/${PROJECT_NAME})  
  
# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
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
