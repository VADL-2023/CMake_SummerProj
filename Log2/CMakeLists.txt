include_directories(include)

add_library(Log2
        src/Log2.cpp
        )

target_link_libraries(Log2 VN)
target_include_directories(Log2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
