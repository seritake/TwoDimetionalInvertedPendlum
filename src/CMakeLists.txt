add_library(Robot STATIC
        Robot.cpp
        JoyStick.cpp
        serial.cc
        impl/unix.cc
        impl/list_ports/list_ports_linux.cc
        impl/list_ports/list_ports_osx.cc
        trackModule/colorTracker.cpp
        )

find_package(OpenCV REQUIRED)

# インクルードパスを指定
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(Robot
        PUBLIC ${PROJECT_SOURCE_DIR}/include
        )