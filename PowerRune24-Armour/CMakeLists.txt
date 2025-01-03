# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(DEBUG_VERSION "DebugMode")
set(RELEASE_VERSION "ReleaseMode")
set(COMPILE_MODE ${RELEASE_VERSION})               # 编译模式
string(TIMESTAMP COMPILE_TIME %y%m%d%H%M)        # 生成编译时间
set(VERSION_MAJOR  1)    # 一级版本号
set(VERSION_MINOR  0)    # 二级版本号
set(VERSION_LEVEL3 5)    # 三级版本号

# Debug模式（目标：上一个版本号+编译时间）
if(${COMPILE_MODE} MATCHES ${DEBUG_VERSION})   
    message(STATUS "Compile mode: ${COMPILE_MODE}")
    set(PROJECT_VER v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_LEVEL3}-${COMPILE_TIME}-debug)
# Release模式（目标：版本号）
else()
    message(STATUS "Compile mode: ${COMPILE_MODE}")
    set(PROJECT_VER v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_LEVEL3})
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PowerRune24-Armour)