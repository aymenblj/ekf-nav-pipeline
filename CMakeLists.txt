cmake_minimum_required(VERSION 3.10)
project(ShadowPathMatcher)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Suppress warnings
if(MSVC)
    add_compile_options(/w)
else()
    add_compile_options(-w)
endif()

# Add Eigen submodule
add_subdirectory(external/eigen)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Find all source files in the src directory
file(GLOB SRC_FILES src/*.cpp include/*.hpp)

add_executable(ShadowPathMatcher ${SRC_FILES})

# Eigen is header-only, so just link include dirs
target_include_directories(ShadowPathMatcher PRIVATE external/eigen)
