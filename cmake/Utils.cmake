################################################################################
# Appends items to a cached list.
macro(append_to_cached_string _string _cacheDesc)
  foreach(newItem ${ARGN})
    set(${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  endforeach(newItem ${ARGN})
endmacro(append_to_cached_string)

################################################################################
# Appends items to a cached list.
macro(append_to_cached_list _list _cacheDesc)
  set(tempList ${${_list}})
  foreach(newItem ${ARGN})
    list(APPEND tempList ${newItem})
  endforeach(newItem ${newItem})
  set(${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
endmacro(append_to_cached_list)

#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
macro(list_to_string _string _list)
    set(${_string})
    foreach(_item ${_list})
      set(${_string} "${${_string}} ${_item}")
    endforeach(_item)
endmacro(list_to_string)

#################################################
# Macro to setup supported compiler warnings
# Based on work of Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST.
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_warnings)
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
endmacro()
