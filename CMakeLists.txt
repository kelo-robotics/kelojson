cmake_minimum_required(VERSION 3.10.2)
project(kelojson_loader)

add_compile_options(
    -std=c++11
    -Wall
    -Wextra
    -Wno-unused-variable
    -Wno-deprecated-copy
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
        kelojson_loader
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

add_library(kelojson_loader
    src/KelojsonAreasLayer.cpp
    src/KelojsonLayer.cpp
    src/KelojsonMap.cpp
    src/KelojsonTypes.cpp
    src/KelojsonTopologyLayer.cpp
    src/KelojsonZonesLayer.cpp
    src/OsmPrimitives.cpp
)

target_link_libraries(kelojson_loader
    ${catkin_LIBRARIES}
)
