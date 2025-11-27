cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set (CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(tinny_data_collect)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/GetDistance2D/inc
  ${PROJECT_BINARY_DIR}
  /usr/local/include/urg_cpp
  /usr/local/include
  /usr/include/lua5.4
)

link_directories(${PROJECT_SOURCE_DIR}
  /usr/local/lib)

set (DEFAULT_LOG_DIR ${PROJECT_SOURCE_DIR})
set (DEFAULT_ROOT ${PROJECT_SOURCE_DIR})
set (SRC_DIR ${PROJECT_SOURCE_DIR}/src)

configure_file (
    "${PROJECT_SOURCE_DIR}/Config.h.in"
    "${PROJECT_BINARY_DIR}/Config.h"
    )

find_package (OpenCV REQUIRED)

FILE(GLOB URG2D_FILES ${PROJECT_SOURCE_DIR}/src/GetDistance2D/src/*.cpp)

# SDL2のパッケージを探す
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
# インクルードディレクトリとライブラリディレクトリの設定
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

#### 実行ファイルの作成
add_executable (tinny_data_collect main.cpp
  ${SRC_DIR}/Urg2d.cpp
  ${URG2D_FILES}
  )
target_link_libraries(tinny_data_collect yaml-cpp ${OpenCV_LIBRARIES} urg_cpp ncurses ${SDL2_LIBRARIES} ${CURSES_LIBRARIES})
