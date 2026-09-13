cmake_minimum_required(VERSION 3.16)
project(UltimateMotion VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)
qt_standard_project_setup()

# the trajectory core, plain C99 so it can be dropped into a controller as it is
add_library(motioncore STATIC
    src/TrajectoryGenerator.h
    src/TrajectoryGenerator.c
)

target_include_directories(motioncore PUBLIC src)

set_target_properties(motioncore PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)      # plain C, nothing for moc to do

# -fno-math-errno drops the errno branch around every sqrt; no result changes, unlike -ffast-math,
# which would reassociate the formulas. the optimisation level itself comes from the build type
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motioncore PRIVATE -fno-math-errno)
endif()

if(UNIX)
    target_link_libraries(motioncore PUBLIC m)
endif()

qt_add_executable(onlinetest
    src/online_main.cpp
    src/OnlineWindow.h
    src/OnlineWindow.cpp
)

target_link_libraries(onlinetest PRIVATE motioncore Qt6::Widgets Qt6::Charts)
