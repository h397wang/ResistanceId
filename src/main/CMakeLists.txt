
cmake_minimum_required(VERSION 2.8)
project( ResistanceId )
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}  SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

