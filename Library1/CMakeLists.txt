include_directories(include)
add_library(Library1 src/foo.cpp)
target_include_directories(Library1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)