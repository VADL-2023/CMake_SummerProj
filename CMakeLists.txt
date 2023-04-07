cmake_minimum_required(VERSION 2.4)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(myProject)

add_subdirectory(Library1)
add_subdirectory(VectorNav)
add_subdirectory(pigpio-master)
add_subdirectory(IMU) 
add_subdirectory(Log)



#Add libraries above main Driver (have to be created to link)
add_subdirectory(MainDriver)
