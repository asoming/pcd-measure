cmake_minimum_required(VERSION 3.16)

project(pcd_measure VERSION 1.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Qt5 5.12 REQUIRED COMPONENTS Core Widgets Concurrent Test)
find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree visualization)
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
  src/main_window.cpp
  src/main_window.h
)
target_include_directories(pcd_measure PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
target_link_libraries(pcd_measure PRIVATE
  pcd_cloud_core
  Qt5::Widgets
  Qt5::Concurrent
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
  add_executable(pcd_gui_test
    tests/main_window_test.cpp
    src/main_window.cpp
    src/main_window.h
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
endif()
