cmake_minimum_required(VERSION 3.8)
project(gui_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_types REQUIRED)

find_package(SDL2 REQUIRED CONFIG REQUIRED COMPONENTS SDL2)
find_package(SDL2 REQUIRED CONFIG COMPONENTS SDL2main)

# imgui
set(IMGUI_SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/imgui_demo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/imgui_draw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/imgui_tables.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/imgui_widgets.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/imgui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/backends/imgui_impl_sdl2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/backends/imgui_impl_sdlrenderer2.cpp
)

# implot
set(IMPLOT_SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/implot/implot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/implot/implot_items.cpp
)

add_executable(main ${IMGUI_SRC_FILES} ${IMPLOT_SRC_FILES} ${CUSTOMS} src/test.cpp)

ament_target_dependencies(main rclcpp sensor_msgs std_msgs custom_types)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/implot)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/GUIBaseCpp/include/imgui/backends)

target_compile_features(main PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

# SDL2::SDL2main may or may not be available. It is e.g. required by Windows GUI applications
if(TARGET SDL2::SDL2main)
    # It has an implicit dependency on SDL2 functions, so it MUST be added before SDL2::SDL2 (or SDL2::SDL2-static)
    target_link_libraries(main SDL2::SDL2main)
endif()

# Link to the actual SDL2 library. SDL2::SDL2 is the shared SDL library, SDL2::SDL2-static is the static SDL library.
target_link_libraries(main SDL2::SDL2)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
