project(SFERA_Omnitracking)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost COMPONENTS program_options REQUIRED)

find_package(MetavisionSDK 4.6.2 EXACT COMPONENTS base core driver cv ui analytics REQUIRED)

add_executable(SFERA_viewer src/SFERA_viewer.cpp)
target_link_libraries(SFERA_viewer MetavisionSDK::base MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui MetavisionSDK::cv MetavisionSDK::analytics Boost::program_options ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(SFERA_omnitracking src/SFERA_omnitracking.cpp)
target_link_libraries(SFERA_omnitracking MetavisionSDK::base MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui MetavisionSDK::cv MetavisionSDK::analytics Boost::program_options ${OpenCV_LIBS} ${PCL_LIBRARIES})



