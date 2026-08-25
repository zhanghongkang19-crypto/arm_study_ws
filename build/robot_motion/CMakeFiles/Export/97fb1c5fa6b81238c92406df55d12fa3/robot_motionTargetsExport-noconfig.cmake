#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_motion::robot_motion" for configuration ""
set_property(TARGET robot_motion::robot_motion APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robot_motion::robot_motion PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobot_motion.so"
  IMPORTED_SONAME_NOCONFIG "librobot_motion.so"
  )

list(APPEND _cmake_import_check_targets robot_motion::robot_motion )
list(APPEND _cmake_import_check_files_for_robot_motion::robot_motion "${_IMPORT_PREFIX}/lib/librobot_motion.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
