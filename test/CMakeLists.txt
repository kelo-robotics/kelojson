# NOTE: all test must end with "_test.cpp" !
FILE(GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp" )

set(KELOJSON_TEST_MAP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/map/two_rooms/two_rooms.kelojson")
ADD_DEFINITIONS(-DKELOJSON_TEST_MAP_FILE=${KELOJSON_TEST_MAP_FILE})

catkin_add_gtest(kelojson_loader_test
    main.cpp
    ${TEST_SOURCES}
)
target_link_libraries(kelojson_loader_test
    ${catkin_LIBRARIES}
    kelojson_loader
)
