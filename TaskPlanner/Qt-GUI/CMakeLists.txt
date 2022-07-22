cmake_minimum_required(VERSION 3.14)

project(AreaPlanner LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick REQUIRED)
find_package(Armadillo REQUIRED)
include_directories(
    include
    include/wykobi
    ${ARMADILLO_INCLUDE_DIRS}
    )
set(SOURCE_FILES
    background.cpp
    background.h
    include/solver_interface.h
    include/algo/taskallocation.h
    include/problem_definition.h
    src/solver_interface.cpp
    src/HorizontalAreaDecomposition.cpp
    src/TriangularDecomposition.cpp
    src/SweepPathCalipers.cpp
    src/MinTurnDecomposition.cpp
    src/RotatingCalipers.cpp
    src/taskallocation.cpp
    )

add_executable(AreaPlanner
  main.cpp
  qml.qrc
  ${SOURCE_FILES}
)

target_compile_definitions(AreaPlanner
  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(AreaPlanner
  PRIVATE Qt${QT_VERSION_MAJOR}::Core
  ${ARMADILLO_LIBRARIES}
  Qt${QT_VERSION_MAJOR}::Quick)
