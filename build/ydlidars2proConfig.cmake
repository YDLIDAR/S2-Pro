set(ydlidar_s2pro_INCLUDE_DIRS "/usr/local/include/ydlidar_s2pro")
set(ydlidar_s2pro_LIBS  /usr/local/lib)
message("CMAKE_PROJECT_NAME: " ${CMAKE_PROJECT_NAME})
message("[config]header: " "/usr/local/include/ydlidar_s2pro")
get_filename_component(_dir "${CMAKE_CURRENT_LIST_FILE}" PATH)
include("${_dir}/./ydlidar_s2pro.cmake")
set(ydlidar_s2pro_LIBS "ydlidar_s2pro")