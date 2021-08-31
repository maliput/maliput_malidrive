##############################################################################
# Configured
##############################################################################

configure_file(version.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
)

##############################################################################
# Installs
##############################################################################

# Files
install(
  FILES
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

##############################################################################
# Directories
##############################################################################

add_subdirectory(builder)
add_subdirectory(common)
add_subdirectory(loader)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
