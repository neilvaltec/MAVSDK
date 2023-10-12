cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(multiple_drones_add_on_the_fly)

add_executable(multiple_drones_add_on_the_fly
    multiple_drones_add_on_the_fly.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(multiple_drones_add_on_the_fly
    MAVSDK::mavsdk
    Threads::Threads
)

if(NOT MSVC)
    add_compile_options(multiple_drones_add_on_the_fly PRIVATE -Wall -Wextra)
else()
    add_compile_options(multiple_drones_add_on_the_fly PRIVATE -WX -W2)
endif()
