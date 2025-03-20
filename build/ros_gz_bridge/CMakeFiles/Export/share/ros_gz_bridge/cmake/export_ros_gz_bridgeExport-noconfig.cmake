#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros_gz_bridge::ros_gz_bridge" for configuration ""
set_property(TARGET ros_gz_bridge::ros_gz_bridge APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ros_gz_bridge::ros_gz_bridge PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libros_gz_bridge.so"
  IMPORTED_SONAME_NOCONFIG "libros_gz_bridge.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros_gz_bridge::ros_gz_bridge )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros_gz_bridge::ros_gz_bridge "${_IMPORT_PREFIX}/lib/libros_gz_bridge.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
