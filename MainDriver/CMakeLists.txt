include_directories(include)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


add_executable(myProject src/main.cpp)
target_link_libraries(myProject Library1 VN IMU pigpio) #no log rn

