cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lola-ar)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED)
find_package(am2b-arvis CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("src")

file(GLOB lola_log_reader_src src/LolaLogReader/main.cpp)
add_executable(lola_log_reader ${lola_log_reader_src})
target_link_libraries(lola_log_reader ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
set_target_properties(lola_log_reader PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

