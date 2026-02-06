cmake_minimum_required(VERSION 3.16)

project(path_generate_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(path_generate_shared SHARED
  src/api.cpp
  src/planner.cpp
  src/geometry.cpp
  src/kinematics.cpp
)

target_include_directories(path_generate_shared
  PUBLIC
    include
    third_party
)

target_compile_definitions(path_generate_shared
  PRIVATE
    PATH_GENERATE_BUILD_DLL
)

option(PATH_GENERATE_BUILD_CLI "Build CLI executable" OFF)

if(PATH_GENERATE_BUILD_CLI)
  add_executable(path_generate_cpp
    src/main.cpp
    src/cli.cpp
    src/planner.cpp
    src/geometry.cpp
    src/kinematics.cpp
  )

  target_include_directories(path_generate_cpp
    PRIVATE
      include
      third_party
  )
endif()
