cmake_minimum_required(VERSION 3.6)
project(tensorrtbuffer)

set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

find_package(CUDA REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

if(EXISTS "/usr/local/include/tensorrt")
    message(STATUS "tensorrt include directory: /usr/local/include/tensorrt")
    link_directories(/usr/local/tensorrt/lib)
    include_directories(/usr/local/include/tensorrt)
else()
    message(STATUS "external path for tensorrt not found")
endif()

# Build artifacts must stay in the active build tree.  The source tree may
# contain a prebuilt library for a different CPU architecture, which must not
# be mistaken for the output of this target on Jetson/aarch64.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

add_library(tensorrtbuffer SHARED src/logger.cpp src/sample_options.cpp src/sample_utils.cpp)


