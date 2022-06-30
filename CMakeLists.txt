cmake_minimum_required(VERSION 3.5.1)
project(kelojson)

add_compile_options(
    -std=c++11
    -Wall
    -Wextra
)

find_package(catkin REQUIRED COMPONENTS
    geometry_common
    yaml_common
)

catkin_package(
    CATKIN_DEPENDS
        geometry_common
        yaml_common
    INCLUDE_DIRS
        include
    LIBRARIES
        kelojson
)

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(kelojson
    # osm
    src/osm/Primitive.cpp
    src/osm/NodePrimitive.cpp
    src/osm/WayPrimitive.cpp
    src/osm/RelationPrimitive.cpp
    src/osm/PrimitiveFactory.cpp
    src/osm/PrimitiveUtils.cpp

    # layer
    src/layer/LayerFactory.cpp

    # areas
    src/layer/areas/AreasLayer.cpp
    src/layer/areas/Area.cpp
    src/layer/areas/Transition.cpp
    src/layer/areas/AreaPlanner.cpp

    # zones
    src/layer/zones/ZonesLayer.cpp
    src/layer/zones/Zone.cpp
    src/layer/zones/NodeZone.cpp
    src/layer/zones/PolygonZone.cpp
    src/layer/zones/PolylineZone.cpp
    src/layer/zones/RampZone.cpp
    src/layer/zones/LoadParkingZone.cpp
    src/layer/zones/TransferStationZone.cpp
    src/layer/zones/OcclusionZone.cpp

    # topology
    src/layer/topology/TopologyLayer.cpp
    src/layer/topology/TopologyNode.cpp
    src/layer/topology/TopologyEdge.cpp

    # occupancy
    src/layer/occupancy/OccupancyGridLayer.cpp
    src/layer/occupancy/OccupancyGrid.cpp

    src/Map.cpp
)

target_link_libraries(kelojson
    ${catkin_LIBRARIES}
)

# =====
# TESTS
# =====
if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif ()

# =======
# INSTALL
# =======
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
