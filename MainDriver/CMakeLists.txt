include_directories(include)


add_executable(myProject src/main.cpp)
target_link_libraries(myProject Library1 VN IMU LOG)

