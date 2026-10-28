cmake_minimum_required(VERSION 3.16)
project(ethercat_controller LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wno-error=pedantic)
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(include/soem)

add_executable(EtherCATController
  src/main.cpp
  include/Controller/controller.cpp
  include/Controller/TrajectoryGenerator.cpp
  include/MotorController/GVdrive.cpp
)

target_include_directories(EtherCATController PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Controller
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MotorController
)

target_link_libraries(EtherCATController PRIVATE
  soem
  Threads::Threads
  yaml-cpp
  Eigen3::Eigen
  rt
)
