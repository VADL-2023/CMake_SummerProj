cmake_minimum_required(VERSION 2.4)

project(myProject)

add_subdirectory(Library1)
add_subdirectory(VectorNav)
add_subdirectory(IMU) #Add libraries above main Driver (have to be created to link)
add_subdirectory(LOG)
add_subdirectory(MainDriver)
