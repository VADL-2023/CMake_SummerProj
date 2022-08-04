include_directories(include)

add_library(IMU src/IMU.cpp)
target_link_libraries(IMU VN)
target_include_directories(IMU PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)