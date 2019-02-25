project(Predictor_particular_approch_cpp)
cmake_minimum_required(VERSION 2.8)


find_package(Boost 1.58.0 COMPONENTS filesystem system iostreams REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(SOURCES robot.cpp Kalman.cpp robot.h Kalman.h)



add_executable(Main Main.cpp)
target_link_libraries(Main ${SOURCES} ${Boost_LIBRARIES})
