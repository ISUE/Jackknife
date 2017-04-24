#
#
#
cmake_minimum_required (VERSION 2.6)
project (Jackknife)

set(Jackknife_Root "${PROJECT_SOURCE_DIR}/../")

message("PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
message("PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")

#
# set up include-directories
#
include_directories(
  "${PROJECT_SOURCE_DIR}"
  "${PROJECT_SOURCE_DIR}/jackknife/"
  "${PROJECT_SOURCE_DIR}/evaluate/"
  "${PROJECT_BINARY_DIR}")

# Add Windows hacks if necessary
if(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  include_directories(
        "${PROJECT_SOURCE_DIR}/winhacks/"
  )
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

configure_file(evaluate/session.cpp.in session.cpp)
configure_file(evaluate/user_independent.cpp.in user_independent.cpp)

set(SOURCES
    jackknife/jackknife_train.cpp
    evaluate/dataset.cpp
    "${PROJECT_BINARY_DIR}/session.cpp"
    "${PROJECT_BINARY_DIR}/user_independent.cpp"
    main.cpp
)

#
# add the main Jackknife executable
#
add_executable(Jackknife ${SOURCES})

#
#
#
if(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    set(CMAKE_CXX_FLAGS "/D_USE_MATH_DEFINES /MP /DNOMINMAX /EHsc")
else()
    set(CMAKE_CXX_FLAGS "-std=c++11 -D_USE_MATH_DEFINES -Wno-deprecated-declarations")
endif()

#
# Turn on optimizations
#
set(CMAKE_BUILD_TYPE Release)
