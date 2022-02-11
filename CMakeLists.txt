cmake_minimum_required(VERSION 3.7)

project(HDxGB)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(HDxGB src/main.cpp)
target_link_libraries(HDxGB ${SDL2_LIBRARIES})

target_compile_options(HDxGB PRIVATE -Werror -Wall -Wextra)