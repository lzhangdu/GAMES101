cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR "C:/Users/Royal/Desktop/GAMES101/hws/opencv/build")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("C:/Users/Royal/Desktop/GAMES101/hws/eigen-3.4.0")
include_directories(.)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)