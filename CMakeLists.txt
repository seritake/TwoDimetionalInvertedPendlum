cmake_minimum_required(VERSION 3.5)
add_subdirectory(src)
add_executable(Main main.cpp)
target_link_libraries(Main Robot)