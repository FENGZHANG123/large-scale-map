 cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
 
 project(large_scale_map)
 
 find_package(PCL REQUIRED)
 
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

#add_executable (dat_test dat_io.h dat_io.cpp dat_io_test.cpp tic_toc.h)
#target_link_libraries (dat_test ${PCL_LIBRARIES})

#add_executable(block_test block.h block.cpp block_test.cpp)
#target_link_libraries(block_test ${PCL_LIBRARIES})

#add_executable(keyframe_test keyframe.h keyframe.cpp keyfram_test.cpp)
#target_link_libraries(keyframe_test ${PCL_LIBRARIES})

add_executable(local_map_test local_map.h local_map.cpp local_map_test.cpp block.h block.cpp keyframe.h keyframe.cpp
    dat_io.h dat_io.cpp tic_toc.h)
target_link_libraries(local_map_test ${PCL_LIBRARIES})
