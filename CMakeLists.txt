cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lola-ar)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED)
find_package(am2b-arvis CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("src")
include_directories("src/iface/include")
include_directories("extern/tclap/include")

# Lola log player
file(GLOB lola_playback_src
    src/LolaPlayback/main.cpp
    src/StepPlannerLogReader.cpp
)
add_executable(lola_playback ${lola_playback_src})
target_link_libraries(lola_playback ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
set_target_properties(lola_playback PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Lola UDP Client
file(GLOB lola_listener_src
    src/LolaListener/main.cpp
)
add_executable(lola_listener ${lola_listener_src})
target_link_libraries(lola_listener ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
set_target_properties(lola_listener PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Aruco marker tracking test
file(GLOB aruco_tracker_test_src
    src/MarkerTracker/main.cpp
)
add_executable(aruco_tracker ${aruco_tracker_test_src})
target_link_libraries(aruco_tracker ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
set_target_properties(aruco_tracker PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Generator for aruco marker boards
file(GLOB aruco_board_generator_src
    src/ArucoBoardGenerator/main.cpp
)
add_executable(aruco_generator ${aruco_board_generator_src})
target_link_libraries(aruco_generator ${OpenCV_LIBS})
set_target_properties(aruco_generator PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
