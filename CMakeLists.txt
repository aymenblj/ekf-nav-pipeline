cmake_minimum_required(VERSION 3.16)
project(NavStream VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pull dependencies (ImGui, ImPlot, GLFW, Eigen, etc.)
include(${CMAKE_SOURCE_DIR}/dependencies/pullDependencies.cmake)

# Project source/header files
file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE SRC_HPP CONFIGURE_DEPENDS include/*.hpp)

# ImGui/ImPlot sources (add their backends as well)
set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
set(IMPLOT_SOURCES
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)

# Eigen include handling (header-only)
# eigen_SOURCE_DIR is set by FetchContent in pullDependencies.cmake and should point to the directory containing Eigen/Dense
set(EXTRA_INCLUDES "${eigen_SOURCE_DIR}")

add_executable(${PROJECT_NAME}
    ${SRC_CPP}
    ${SRC_HPP}
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
    ${glfw_SOURCE_DIR}/include
    ${EXTRA_INCLUDES}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        glfw
)

# Link OpenGL on Windows
if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE opengl32)
endif()

# Link pthread on Linux
if (UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()

# Automatic Linking for Threads
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# Enable Doxygen docs if available
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    add_custom_target(doc_doxygen
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT} > doxygen.log 2>&1
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen (see doxygen.log for warnings)"
        VERBATIM
    )
else()
    message(STATUS "Doxygen not found, 'doc_doxygen' target will not be available.")
endif()