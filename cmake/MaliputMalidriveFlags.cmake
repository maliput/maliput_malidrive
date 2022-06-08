#####################################
# Handle CFlags

unset (CMAKE_C_FLAGS_ALL CACHE)
unset (CMAKE_CXX_FLAGS CACHE)

include(${PROJECT_SOURCE_DIR}/cmake/DefaultCFlags.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SanitizersConfig.cmake)

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
list(APPEND WARN_LEVEL -Waddress -Warray-bounds -Wcomment -Wformat -Wnonnull)
list(APPEND WARN_LEVEL -Wparentheses -Wreorder -Wreturn-type)
list(APPEND WARN_LEVEL -Wsequence-point -Wsign-compare -Wstrict-aliasing)
list(APPEND WARN_LEVEL -Wstrict-overflow=1 -Wswitch -Wtrigraphs -Wuninitialized)
list(APPEND WARN_LEVEL -Wunused-function -Wunused-label -Wunused-value)
list(APPEND WARN_LEVEL -Wunused-variable -Wvolatile-register-var)

# Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
set(UNFILTERED_FLAGS "-Wc++17-compat")

filter_valid_compiler_warnings(${WARN_LEVEL} -Wextra -Wno-long-long
  -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value
  -Winit-self -Wswitch-default
  -Wmissing-include-dirs -Wno-pragmas)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS} ${UNFILTERED_FLAGS}")

# Report out
message(STATUS "Shared Libraries..............${BUILD_SHARED_LIBS}")
message(STATUS "Build Type....................${CMAKE_BUILD_TYPE}")
message(STATUS "Install path..................${CMAKE_INSTALL_PREFIX}")
if (DEFINED CMAKE_CXX_FLAGS)
  message(STATUS "Custom CFlags.................${CMAKE_CXX_FLAGS}")
else()
  message (STATUS "Using default CFlags")
endif()

message(STATUS "\n----------------------------------------\n")
