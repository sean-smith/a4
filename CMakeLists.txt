cmake_minimum_required(VERSION 2.8)
project( Homework4 )
find_package( OpenCV REQUIRED )
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(SOURCES_BAT Tracker.cpp Tracker.h Kalman.cpp Kalman.h Hungarian.cpp Hungarian.h Batman_KF.cpp)
#add_executable( bats_ab_greedy bats_abg.cpp )
add_executable( Batman_KF ${SOURCES_BAT} )
add_executable( Batman_ABGreedy Batman_ABGreedy.cpp )
target_link_libraries( Batman_KF ${OpenCV_LIBS} )
target_link_libraries( Batman_ABGreedy ${OpenCV_LIBS} )
