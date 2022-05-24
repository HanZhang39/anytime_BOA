project (Multi_objective_search)


cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET (CMAKE_LINKER  "/usr/bin/ld")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")

include_directories("inc")

file(GLOB SOURCES "src/BiCriteria/*.cpp" "src/Utils/*.cpp")
# add_library(foo ...)

# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories( ${Boost_INCLUDE_DIRS} )

# find_package(nlohmann_json 3.2.0 REQUIRED)

message(${CMAKE_BINARY_DIR})

add_executable(solver src/driver.cpp ${SOURCES})
target_link_libraries(solver ${Boost_LIBRARIES})

