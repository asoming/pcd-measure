cmake_minimum_required(VERSION 3.16)

project(pcd_measure VERSION 2.3.1 LANGUAGES C CXX)

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

add_library(pcd_cloud_core STATIC
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
target_include_directories(pcd_cloud_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd_cloud_core PUBLIC
  Qt5::Core
  ${PCL_LIBRARIES}
)
target_compile_options(pcd_cloud_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pcd_measure
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
target_include_directories(pcd_measure PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
target_link_libraries(pcd_measure PRIVATE
  pcd_cloud_core
  Qt5::Widgets
  Qt5::Concurrent
  Qt5::PrintSupport
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)
target_compile_options(pcd_measure PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pcd_analyze src/analyze_cli.cpp)
target_link_libraries(pcd_analyze PRIVATE pcd_cloud_core Qt5::Core)
target_compile_options(pcd_analyze PRIVATE -Wall -Wextra -Wpedantic)

if(COMMAND vtk_module_autoinit)
  vtk_module_autoinit(
    TARGETS pcd_measure
    MODULES ${VTK_LIBRARIES}
  )
endif()

install(TARGETS pcd_measure pcd_analyze
  RUNTIME DESTINATION bin
)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME pcd_rosbag_python_test
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools"
      "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/tests/rosbag_diag"
      -p "test_*.py"
  )

  add_test(
    NAME pcd_olx_record_python_test
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools"
      "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/tests/olx_record"
      -p "test_*.py"
  )

  add_executable(pcd_rosbag_tools_test tests/rosbag_tools_test.cpp)
  target_include_directories(pcd_rosbag_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(pcd_rosbag_tools_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_rosbag_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_rosbag_tools_test COMMAND pcd_rosbag_tools_test)

  add_executable(pcd_rosbag_dialog_test
    tests/rosbag_dialog_test.cpp
    src/rosbag_dialog.cpp
    src/rosbag_dialog.h
  )
  target_compile_definitions(pcd_rosbag_dialog_test PRIVATE
    PCD_MEASURE_TEST_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_include_directories(pcd_rosbag_dialog_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(pcd_rosbag_dialog_test PRIVATE
    pcd_cloud_core
    Qt5::Widgets
    Qt5::Test
  )
  target_compile_options(pcd_rosbag_dialog_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_rosbag_dialog_test COMMAND pcd_rosbag_dialog_test)

  add_test(
    NAME pcd_rosbag_playback_test
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/rosbag_playback_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  add_executable(pcd_real_workflow_test tests/real_workflow_test.cpp)
  target_include_directories(pcd_real_workflow_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_real_workflow_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_real_workflow_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_real_workflow_test COMMAND pcd_real_workflow_test)

  add_executable(pcd_cloud_data_test tests/cloud_data_test.cpp)
  target_compile_definitions(pcd_cloud_data_test PRIVATE
    PCD_MEASURE_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
  )
  target_include_directories(pcd_cloud_data_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_cloud_data_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_cloud_data_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_cloud_data_test COMMAND pcd_cloud_data_test)

  add_executable(pcd_transform_tools_test tests/transform_tools_test.cpp)
  target_include_directories(pcd_transform_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_transform_tools_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_transform_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_transform_tools_test COMMAND pcd_transform_tools_test)

  add_executable(pcd_comparison_tools_test tests/comparison_tools_test.cpp)
  target_include_directories(pcd_comparison_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_comparison_tools_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_comparison_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_comparison_tools_test COMMAND pcd_comparison_tools_test)

  add_executable(pcd_analysis_tools_test tests/analysis_tools_test.cpp)
  target_include_directories(pcd_analysis_tools_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_analysis_tools_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_analysis_tools_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_analysis_tools_test COMMAND pcd_analysis_tools_test)

  add_executable(pcd_measurement_test tests/measurement_test.cpp)
  target_include_directories(pcd_measurement_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_measurement_test PRIVATE
    pcd_cloud_core
    Qt5::Test
  )
  target_compile_options(pcd_measurement_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_measurement_test COMMAND pcd_measurement_test)

  add_executable(pcd_gui_test
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
  target_compile_definitions(pcd_gui_test PRIVATE
    PCD_MEASURE_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
  )
  target_include_directories(pcd_gui_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
  )
  target_link_libraries(pcd_gui_test PRIVATE
    pcd_cloud_core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Test
    Qt5::PrintSupport
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
  )
  target_compile_options(pcd_gui_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pcd_gui_test COMMAND pcd_gui_test)
  if(COMMAND vtk_module_autoinit)
    vtk_module_autoinit(
      TARGETS pcd_gui_test
      MODULES ${VTK_LIBRARIES}
    )
  endif()

  add_test(
    NAME pcd_packaging_test
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/packaging_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}" "$<TARGET_FILE:pcd_analyze>"
      "$<TARGET_FILE:pcd_measure>"
  )
endif()
