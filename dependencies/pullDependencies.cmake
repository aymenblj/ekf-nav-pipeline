include(FetchContent)

set(EXTERNAL_LOCATION ${CMAKE_BINARY_DIR}/external)

# ImGui
set(IMGUI_LOCATION ${EXTERNAL_LOCATION}/imgui)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.90.4
  SOURCE_DIR     ${IMGUI_LOCATION}/src
  BINARY_DIR     ${IMGUI_LOCATION}/build
  SUBBUILD_DIR   ${IMGUI_LOCATION}/subbuild
)
set(IMGUI_BUILD_DOCS OFF CACHE BOOL "Don't build ImGui docs")
FetchContent_MakeAvailable(imgui)

# ImPlot
set(IMPLOT_LOCATION ${EXTERNAL_LOCATION}/implot)
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        v0.16
  SOURCE_DIR     ${IMPLOT_LOCATION}/src
  BINARY_DIR     ${IMPLOT_LOCATION}/build
  SUBBUILD_DIR   ${IMPLOT_LOCATION}/subbuild
)
set(IMPLOT_BUILD_DOCS OFF CACHE BOOL "Don't build ImPlot docs")
FetchContent_MakeAvailable(implot)

# GLFW
set(GLFW_LOCATION ${EXTERNAL_LOCATION}/glfw)
set(GLFW_INSTALL OFF CACHE BOOL "Disable GLFW install/uninstall targets" FORCE)
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.3.8
  SOURCE_DIR     ${GLFW_LOCATION}/src
  BINARY_DIR     ${GLFW_LOCATION}/build
  SUBBUILD_DIR   ${GLFW_LOCATION}/subbuild
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Don't build GLFW docs")
FetchContent_MakeAvailable(glfw)

# Eigen (header-only)
set(EIGEN_LOCATION ${EXTERNAL_LOCATION}/eigen)
set(EIGEN_DISABLE_INSTALL ON CACHE BOOL "Disable Eigen install/uninstall targets" FORCE)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  SOURCE_DIR     ${EIGEN_LOCATION}/src
  BINARY_DIR     ${EIGEN_LOCATION}/build
  SUBBUILD_DIR   ${EIGEN_LOCATION}/subbuild
)
# set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
# set(EIGEN_BUILD_DOCS OFF CACHE BOOL "Don't build Eigen docs")
FetchContent_MakeAvailable(eigen)

# nlohmann_json (header-only)
set(NLOHMANN_JSON_LOCATION ${EXTERNAL_LOCATION}/nlohmann_json)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
  SOURCE_DIR     ${NLOHMANN_JSON_LOCATION}/src
  BINARY_DIR     ${NLOHMANN_JSON_LOCATION}/build
  SUBBUILD_DIR   ${NLOHMANN_JSON_LOCATION}/subbuild
)
FetchContent_MakeAvailable(nlohmann_json)
