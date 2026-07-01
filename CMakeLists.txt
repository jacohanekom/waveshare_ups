cmake_minimum_required(VERSION 3.16)
project(waveshare_ups CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(waveshare_ups waveshare_ups.cpp)

target_include_directories(waveshare_ups PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(waveshare_ups PRIVATE -Wall -Wextra -O2)

install(TARGETS waveshare_ups DESTINATION bin)
install(FILES config.ini DESTINATION share/waveshare-ups)
