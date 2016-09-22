cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lola-ar)

option(BUILD_LOLA_PLAYBACK "Build lola log playback" FALSE)
option(BUILD_LOLA_LISTENER "Build online lola listener" TRUE)
option(BUILD_MARKER_TRACKER "Build aruco marker tracker" FALSE)
option(BUILD_ARUCO_GENERATOR "Build aruco marker board generator" FALSE)


include_directories("src")
include_directories("src/iface/include")
include_directories("extern/tclap/include")

find_package(am2b-arvis CONFIG REQUIRED)

if (BUILD_LOLA_PLAYBACK)
 find_package(PCL 1.7 REQUIRED)
 find_package( OpenCV REQUIRED)
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

 
  # Lola log player
  file(GLOB lola_playback_src
      src/LolaPlayback/main.cpp
      src/StepPlannerLogReader.cpp
  )
  add_executable(lola_playback ${lola_playback_src})
  target_link_libraries(lola_playback ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
  set_target_properties(lola_playback PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

endif()

if (BUILD_LOLA_LISTENER)
  file(GLOB lola_listener_src
      src/LolaListener/main.cpp
  )
  add_executable(lola_listener ${lola_listener_src})
  target_link_libraries(lola_listener ${am2b-arvis_LIBRARY})
  set_target_properties(lola_listener PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if (BUILD_MARKER_TRACKER)
  # Aruco marker tracking test
  file(GLOB aruco_tracker_test_src
      src/MarkerTracker/main.cpp
  )
  add_executable(aruco_tracker ${aruco_tracker_test_src})
  target_link_libraries(aruco_tracker ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
  set_target_properties(aruco_tracker PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if (BUILD_ARUCO_GENERATOR)
  # Generator for aruco marker boards
  file(GLOB aruco_board_generator_src
      src/ArucoBoardGenerator/main.cpp
  )
  add_executable(aruco_generator ${aruco_board_generator_src})
  target_link_libraries(aruco_generator ${OpenCV_LIBS})
  set_target_properties(aruco_generator PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
