cmake_minimum_required(VERSION 3.7)

project(HDxGB)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(SDL2 REQUIRED)

include_directories(
	"src/"
	${SDL2_INCLUDE_DIR}
)

include_directories(${SDL2_INCLUDE_DIRS})

file(GLOB SOURCES
    src/*.cpp
)

add_executable(HDxGB ${SOURCES})
target_link_libraries(HDxGB ${SDL2_LIBRARY})

message(STATUS "include dir=${SDL2_INCLUDE_DIR}")

target_compile_options(HDxGB PRIVATE)# -Werror -Wall -Wextra)