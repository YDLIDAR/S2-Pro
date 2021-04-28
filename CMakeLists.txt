cmake_minimum_required(VERSION 2.8)
project(ydlidar_s2pro)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#include_directories(include)
#include_directories(src)
SET(CMAKE_INSTALL_FULL_PATH  "/usr/local")
#include_directories(include)
#include_directories(src)
list(APPEND HEADERS include  src)
include_directories(${HEADERS})

FILE(GLOB SDK_INCLUDE
	"src/*.h"
	"include/*.h"
)

IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)

	
ENDIF()


set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")


add_subdirectory(samples)

add_library(ydlidar_s2pro STATIC ${SDK_SRC})
IF (WIN32)
target_link_libraries(ydlidar_s2pro setupapi)
ELSE()
target_link_libraries(ydlidar_s2pro rt pthread)
ENDIF() 
#foreach(HEADER ${HEADERS})
#	message("header:" ${HEADER})
#endforeach()
set(CMAKE_INSTALL_INCLUDE_DIR  ${CMAKE_INSTALL_FULL_PATH}/include/${PROJECT_NAME})
set(CMAKE_INSTALL_LIBS_DIR   /usr/local/lib/)
message("CMAKE_PROJECT_NAME: " ${CMAKE_PROJECT_NAME})
install(FILES ${SDK_INCLUDE} DESTINATION ${CMAKE_INSTALL_INCLUDE_DIR})
#install(FILES ${CMAKE_BINARY_DIR}/lib${PROJECT_NAME}.a  DESTINATION ${CMAKE_INSTALL_LIBS_DIR})
#install(FILES ${CMAKE_SOURCE_DIR}/src/common.h DESTINATION ${CMAKE_INSTALL_FULL_PATH}/include)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_FULL_PATH}/lib EXPORT ${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_FULL_PATH}/lib/cmake/ydlidas2pro)

message("PROJECT_BINARY_DIR:"  ${PROJECT_BINARY_DIR})
configure_file(${CMAKE_SOURCE_DIR}/cmake/ydlidarConfig.cmake.in  ${PROJECT_BINARY_DIR}/ydlidars2proConfig.cmake @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/ydlidars2proConfig.cmake DESTINATION ${CMAKE_INSTALL_FULL_PATH}/lib/cmake/ydlidars2pro)
