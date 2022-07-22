
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

#sudo apt-get install libarmadillo-dev
find_package(Armadillo REQUIRED)

#remove_definitions(-Werror -fno-stack-protector)

include_directories(include
        ${ARMADILLO_INCLUDE_DIR}
        gui/include
        ThirdParty/qcustomplot
        ThirdParty/wykobi
        ${ARMADILLO_INCLUDE_DIRS}
        ThirdParty/json/include
        )

add_subdirectory(gui)
set(AREA_COVERAGE_HEADER_FILES
        include/problem.h
        include/Algorithm/SeqAreaDecompose.h

#        src/Alorithm/SweepPath.cpp
#        include/Algorithm/SweepPath.h
        include/logger.h
        include/Algorithm/MinimumTime.h
        include/Algorithm/HorizontalAreaDecomposition.h
        include/Algorithm/RotatingCalipers.h
        include/Algorithm/MinTurnDecomposition.h
        include/Algorithm/SweepPathCalipers.h
        include/pch.h
        include/Algorithm/TriangleDecomposition.h
        )

set(AREA_COVERAGE_SOURCE_FILES
        src/solver.cpp include/solver.h
        src/Alorithm/SeqAreaDecompose.cpp
        src/Alorithm/MinimumTime.cpp
        src/Alorithm/SweepPathCalipers.cpp
        src/Alorithm/MinTurnDecomposition.cpp
        src/Alorithm/HorizontalAreaDecomposition.cpp
        src/Alorithm/RotatingCalipers.cpp
        )

add_executable(${PROJECT_NAME}_tasking main.cpp
        ${AREA_COVERAGE_SOURCE_FILES}
        ${AREA_COVERAGE_HEADER_FILES})
target_link_libraries(${PROJECT_NAME}_tasking
        -pthread
        ${ARMADILLO_LIBRARIES}
        ${catkin_LIBRARIES}
        gui)
