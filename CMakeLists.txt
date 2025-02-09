cmake_minimum_required(VERSION 3.8)
project(vanttec_controllers)
set(CMAKE_CXX_STANDARD 17)

option(USE_CUDA "Enable CUDA for PyTorch" ON)  # Default to ON
message(STATUS "USE_CUDA is set to: ${USE_CUDA}")

# === Fix: Ensure Only One Version of Torch is Used ===
if (USE_CUDA)
    set(Torch_DIR "/usr/local/libtorch_gpu/libtorch/share/cmake/Torch")
    set(CMAKE_PREFIX_PATH "/usr/local/libtorch_gpu/libtorch")
    
    # Explicitly Set CUDA Architectures for RTX 2060
    set(CMAKE_CUDA_ARCHITECTURES "75")
else()
    set(Torch_DIR "/usr/local/libtorch_cpu/libtorch/share/cmake/Torch")
    set(CMAKE_PREFIX_PATH "/usr/local/libtorch_cpu/libtorch")
endif()

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED CONFIG)

message(STATUS "Torch_DIR is set to: ${Torch_DIR}")
message(STATUS "Torch_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
message(STATUS "Torch_LIBRARIES: ${TORCH_LIBRARIES}")

message(STATUS "vanttec_controllers: CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
message(STATUS "vanttec_controllers: CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "vanttec_controllers: Torch_DIR=${Torch_DIR}")

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Collect all source files recursively
file(GLOB_RECURSE vanttec_controllers_SRC src/*.cpp)

# Define the vanttec_controllers library
add_library(vanttec_controllers "")

# === Include Directories ===
target_include_directories(vanttec_controllers
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/vanttec_controllers>
    ${EIGEN3_INCLUDE_DIR}  # Include Eigen headers
    ${TORCH_INCLUDE_DIRS}  # Include PyTorch headers
)

# === Add Source Files ===
target_sources(vanttec_controllers
  PRIVATE
  src/controllers/control_laws/PID/first_order/pid.cpp
  src/controllers/feedback_linearization/base/fb_lin_control.cpp
  src/controllers/feedback_linearization/PID/first_order/fblin_pid.cpp
  src/controllers/feedback_linearization/model_based/SDCs/regulators/vtec_sdc1_pid.cpp

  src/dynamic_models/ground_vehicles/car_like/base/car_3dof_dynamic_model.cpp
  src/dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.cpp

  src/dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1_hybrid.cpp 

  src/utils/utils.cpp
)

# === Fix: Link Torch Properly ===
target_link_libraries(vanttec_controllers
    Eigen3::Eigen  # Link Eigen3
    # ${TORCH_LIBRARIES}  # Use PyTorch libraries
    ${CMAKE_PREFIX_PATH}/lib/libtorch.so
    ${CMAKE_PREFIX_PATH}/lib/libtorch_cpu.so
    ${CMAKE_PREFIX_PATH}/lib/libc10.so
    ${CMAKE_PREFIX_PATH}/lib/libc10_cuda.so
)

# === Google Test Setup ===
include(GoogleTest)
file(GLOB_RECURSE vanttec_controllers_TESTS tests/*.cpp)
enable_testing()
add_executable(vanttec_controllers_test ${vanttec_controllers_TESTS})
target_include_directories(vanttec_controllers_test PRIVATE src/)
target_link_libraries(vanttec_controllers_test gtest_main vanttec_controllers)

gtest_discover_tests(vanttec_controllers_test)

enable_testing()
