#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "uorover_ik::rmm_hardware" for configuration ""
set_property(TARGET uorover_ik::rmm_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(uorover_ik::rmm_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librmm_hardware.so"
  IMPORTED_SONAME_NOCONFIG "librmm_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS uorover_ik::rmm_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_uorover_ik::rmm_hardware "${_IMPORT_PREFIX}/lib/librmm_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
