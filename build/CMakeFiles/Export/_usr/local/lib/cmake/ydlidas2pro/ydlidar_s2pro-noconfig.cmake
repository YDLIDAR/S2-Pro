#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ydlidar_s2pro" for configuration ""
set_property(TARGET ydlidar_s2pro APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ydlidar_s2pro PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C;CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "rt;pthread"
  IMPORTED_LOCATION_NOCONFIG "/usr/local/lib/libydlidar_s2pro.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ydlidar_s2pro )
list(APPEND _IMPORT_CHECK_FILES_FOR_ydlidar_s2pro "/usr/local/lib/libydlidar_s2pro.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
