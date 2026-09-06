cmake_minimum_required(VERSION 3.16)
project(UltimateMotion VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)
qt_standard_project_setup()

qt_add_executable(braketest
    src/main.cpp
    src/Axis.h
    src/TrajectoryGenerator.h
    src/TrajectoryGenerator.cpp
    src/Profilerold.h
    src/Profilerold.cpp
    src/MainWindow.h
    src/MainWindow.cpp
)

target_link_libraries(braketest PRIVATE Qt6::Widgets Qt6::Charts)

qt_add_executable(onlinetest
    src/online_main.cpp
    src/Axis.h
    src/TrajectoryGenerator.h
    src/TrajectoryGenerator.cpp
    src/OnlineWindow.h
    src/OnlineWindow.cpp
)

target_link_libraries(onlinetest PRIVATE Qt6::Widgets Qt6::Charts)
