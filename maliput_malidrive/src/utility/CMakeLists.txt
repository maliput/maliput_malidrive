##############################################################################
# Malidrive utility
##############################################################################

##############################################################################
# Handle CFlags

include(${PROJECT_SOURCE_DIR}/cmake/MaliputMalidriveFlags.cmake)
##############################################################################

add_library(utility
  file_tools.cc
  resources.cc
)
add_library(maliput_malidrive::utility ALIAS utility)

set_target_properties(utility PROPERTIES OUTPUT_NAME maliput_malidrive_utility)
target_include_directories(
  utility
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS utility
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    file_tools.h
    resources.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/utility
)
