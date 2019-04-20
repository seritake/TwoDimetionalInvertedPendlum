add_library(Robot STATIC
        Robot.cpp
        serial.cc
        impl/unix.cc
        impl/list_ports/list_ports_linux.cc
        impl/list_ports/list_ports_osx.cc
        trackModule/colorTracker.cpp
        )
target_include_directories(Robot
        PUBLIC ${PROJECT_SOURCE_DIR}/include
        )