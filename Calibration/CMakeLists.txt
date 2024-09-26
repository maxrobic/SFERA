project(calibration)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

find_package(MetavisionSDK 4.6.2 EXACT COMPONENTS core driver cv calibration ui REQUIRED)
find_package(Boost COMPONENTS filesystem program_options REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data_img)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data_point)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/datasorted)

set(source_files src/step_one_primitive_detection_recording.cpp src/blinking_chessboard_detector.cpp src/blinking_dots_grid_detector.cpp)
add_executable(step_one_primitive_detection_recording ${source_files})
target_link_libraries(step_one_primitive_detection_recording
	MetavisionSDK::core
	MetavisionSDK::base
	MetavisionSDK::driver
	MetavisionSDK::cv
	MetavisionSDK::calibration
	MetavisionSDK::ui
	Boost::filesystem
	Boost::program_options
	${OpenCV_LIBS})

add_executable(step_two_sort src/step_two_sort.cpp)
target_link_libraries(step_two_sort ${OpenCV_LIBS})

add_executable(step_three_omnicalibration src/step_three_omnicalibration.cpp)
target_link_libraries(step_three_omnicalibration ${OpenCV_LIBS})
