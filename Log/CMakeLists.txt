include_directories(include)
add_library(LOG
        src/LOG.cpp
        )

target_link_libraries(LOG IMU)
target_include_directories(LOG PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)