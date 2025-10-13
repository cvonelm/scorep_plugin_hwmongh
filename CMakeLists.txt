cmake_minimum_required(VERSION 3.10)

project(hwmongh_plugin)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PLUGIN_SOURCE src/hwmongh.cpp)

add_subdirectory(lib/scorep_plugin_cxx_wrapper)
add_subdirectory(lib/fmt)

include(cmake/FindScorep.cmake)

if(SCOREP_FOUND)
	include_directories(${SCOREP_INCLUDE_DIRS})
	link_directories(${SCOREP_LIBRARY_DIRS})
else()
	message(SEND_ERROR "Score-P not found!")
endif()

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCE})
target_link_libraries(${PROJECT_NAME} PRIVATE  Scorep::scorep-plugin-cxx fmt::fmt)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
