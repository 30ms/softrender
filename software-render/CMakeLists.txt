# CMakeList.txt: software-render 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# 将源代码添加到此项目的可执行文件。
add_executable (software-render "software-render.cpp" "software-render.h"   "utils.cpp" "utils.h")

#添加子目录

#设置版本变量
set(VERSION_MAJOR 1)
set(VERSION_MINOR 1)

#cmake将输入文件(config.h.in)修改后输出为config.h
configure_file(config.h.in config.h)

#将libgraph库添加到 EXTRA_LIBS 列变量
list(APPEND EXTRA_LIBS libgraph)

target_include_directories(software-render 
                            PUBLIC "${PROJECT_BINARY_DIR}/software-render"  #头文件(config.h)输出到了生成项目目录，添加包含
                           )

target_link_libraries(software-render 
                        PUBLIC ${EXTRA_LIBS})  #链接库


#拷贝文件到输出目录
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/obj/african_head" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/obj")
# TODO: 如有需要，请添加测试并安装目标。
