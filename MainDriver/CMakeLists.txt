include_directories(include)

#if(COMMAND cmake_policy)
#  cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)


add_executable(myProject src/vacuumTest.cpp)
target_link_libraries(myProject Library1 VN IMU pigpio Log)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(myProject ${CMAKE_THREAD_LIBS_INIT})
