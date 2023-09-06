##############################################################################
# Directories
##############################################################################

if(NOT WHEEL_GENERATION)
  add_subdirectory(applications)
endif()
add_subdirectory(maliput_malidrive)
add_subdirectory(plugin)
add_subdirectory(utility)
