cmake_minimum_required(VERSION 3.16)

project(point_cloud_workbench VERSION 2.4.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Qt5 5.12 REQUIRED COMPONENTS Core Widgets Concurrent Test PrintSupport)
find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree filters segmentation registration visualization)
find_package(VTK REQUIRED COMPONENTS
  CommonColor
  CommonCore
  CommonDataModel
  FiltersCore
  FiltersSources
  GUISupportQt
  InteractionStyle
  RenderingCore
  RenderingFreeType
  RenderingOpenGL2
)

add_definitions(${PCL_DEFINITIONS})

add_library(point_cloud_core STATIC
  src/cloud_data.cpp
  src/cloud_data.h
  src/analysis_tools.cpp
  src/analysis_tools.h
  src/comparison_tools.cpp
  src/comparison_tools.h
  src/measurement.cpp
  src/measurement.h
  src/rosbag_tools.cpp
  src/rosbag_tools.h
  src/transform_tools.cpp
  src/transform_tools.h
)
target_include_directories(point_cloud_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(point_cloud_core PUBLIC
  Qt5::Core
  ${PCL_LIBRARIES}
)
target_compile_options(point_cloud_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(point_cloud_workbench
  src/main.cpp
  src/environment_setup_panel.cpp
  src/environment_setup_panel.h
  src/main_window.cpp
  src/main_window.h
  src/olx_capture_dialog.cpp
  src/olx_capture_dialog.h
  src/plot_widget.cpp
  src/plot_widget.h
  src/rosbag_dialog.cpp
  src/rosbag_dialog.h
)
target_include_directories(point_cloud_workbench PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
target_link_libraries(point_cloud_workbench PRIVATE
  point_cloud_core
  Qt5::Widgets
  Qt5::Concurrent
  Qt5::PrintSupport
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)
target_compile_options(point_cloud_workbench PRIVATE -Wall -Wextra -Wpedantic)

add_executable(point_cloud_analyze src/analyze_cli.cpp)
target_link_libraries(point_cloud_analyze PRIVATE point_cloud_core Qt5::Core)
target_compile_options(point_cloud_analyze PRIVATE -Wall -Wextra -Wpedantic)

if(COMMAND vtk_module_autoinit)
  vtk_module_autoinit(
    TARGETS point_cloud_workbench
    MODULES ${VTK_LIBRARIES}
  )
endif()

install(TARGETS point_cloud_workbench point_cloud_analyze
  RUNTIME DESTINATION bin
)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME rosbag_python_test
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools"
      "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/tests/rosbag_diag"
      -p "test_*.py"
  )

  add_test(
    NAME olx_record_python_test
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools"
      "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/tests/olx_record"
      -p "test_*.py"
  )

  add_executable(rosbag_tools_test tests/rosbag_tools_test.cpp)
  target_include_directories(rosbag_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(rosbag_tools_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(rosbag_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME rosbag_tools_test COMMAND rosbag_tools_test)

  add_executable(rosbag_dialog_test
    tests/rosbag_dialog_test.cpp
    src/rosbag_dialog.cpp
    src/rosbag_dialog.h
  )
  target_compile_definitions(rosbag_dialog_test PRIVATE
    POINT_CLOUD_WORKBENCH_TEST_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_include_directories(rosbag_dialog_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(rosbag_dialog_test PRIVATE
    point_cloud_core
    Qt5::Widgets
    Qt5::Test
  )
  target_compile_options(rosbag_dialog_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME rosbag_dialog_test COMMAND rosbag_dialog_test)

  add_test(
    NAME rosbag_playback_test
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/rosbag_playback_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  add_executable(real_workflow_test tests/real_workflow_test.cpp)
  target_include_directories(real_workflow_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(real_workflow_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(real_workflow_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME real_workflow_test COMMAND real_workflow_test)

  add_executable(cloud_data_test tests/cloud_data_test.cpp)
  target_compile_definitions(cloud_data_test PRIVATE
    POINT_CLOUD_WORKBENCH_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
  )
  target_include_directories(cloud_data_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(cloud_data_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(cloud_data_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME cloud_data_test COMMAND cloud_data_test)

  add_executable(transform_tools_test tests/transform_tools_test.cpp)
  target_include_directories(transform_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(transform_tools_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(transform_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME transform_tools_test COMMAND transform_tools_test)

  add_executable(comparison_tools_test tests/comparison_tools_test.cpp)
  target_include_directories(comparison_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(comparison_tools_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(comparison_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME comparison_tools_test COMMAND comparison_tools_test)

  add_executable(analysis_tools_test tests/analysis_tools_test.cpp)
  target_include_directories(analysis_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(analysis_tools_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(analysis_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME analysis_tools_test COMMAND analysis_tools_test)

  add_executable(measurement_test tests/measurement_test.cpp)
  target_include_directories(measurement_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(measurement_test PRIVATE
    point_cloud_core
    Qt5::Test
  )
  target_compile_options(measurement_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME measurement_test COMMAND measurement_test)

  add_executable(point_cloud_workbench_gui_test
    tests/main_window_test.cpp
    src/environment_setup_panel.cpp
    src/environment_setup_panel.h
    src/main_window.cpp
    src/main_window.h
    src/olx_capture_dialog.cpp
    src/olx_capture_dialog.h
    src/plot_widget.cpp
    src/plot_widget.h
    src/rosbag_dialog.cpp
    src/rosbag_dialog.h
  )
  target_compile_definitions(point_cloud_workbench_gui_test PRIVATE
    POINT_CLOUD_WORKBENCH_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
  )
  target_include_directories(point_cloud_workbench_gui_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
  )
  target_link_libraries(point_cloud_workbench_gui_test PRIVATE
    point_cloud_core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Test
    Qt5::PrintSupport
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
  )
  target_compile_options(point_cloud_workbench_gui_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME point_cloud_workbench_gui_test COMMAND point_cloud_workbench_gui_test)
  if(COMMAND vtk_module_autoinit)
    vtk_module_autoinit(
      TARGETS point_cloud_workbench_gui_test
      MODULES ${VTK_LIBRARIES}
    )
  endif()

  add_test(
    NAME packaging_test
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/packaging_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}" "$<TARGET_FILE:point_cloud_analyze>"
      "$<TARGET_FILE:point_cloud_workbench>"
  )
endif()
