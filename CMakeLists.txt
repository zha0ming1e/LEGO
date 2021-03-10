cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(lego)

include(CPack)

# lib prefix
set(LIB_PREFIX lego_)

set(lego_C_FLAGS)
set(lego_CXX_FLAGS)

# built type
#if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release CACHE STRING
#      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#      FORCE)
#endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_BUILD_TYPE "Release")

# postfix
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
set(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# postfix
if(CMAKE_BUILD_TYPE MATCHES Release)
  set(EXE_POSTFIX)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  set(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  set(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
endif(CMAKE_BUILD_TYPE MATCHES Release)

# dynamic or static lib
option (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the lego plugin system)" ON)
set (LEGO_LIB_TYPE STATIC)
if (BUILD_SHARED_LIBS)
  set (LEGO_LIB_TYPE SHARED)
endif ()

# output directories
set(lego_RUNTIME_OUTPUT_DIRECTORY ${lego_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
set(lego_LIBRARY_OUTPUT_DIRECTORY ${lego_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${lego_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${lego_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${lego_RUNTIME_OUTPUT_DIRECTORY})

# installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(INCLUDES_INSTALL_DIR ${INCLUDES_DESTINATION}/lego)

# look for SuiteSparse and Eigen3
list(APPEND CMAKE_MODULE_PATH ${lego_SOURCE_DIR}/cmake_modules)

# os
if (UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on Unix")
endif (UNIX)

option(BUILD_LGPL_SHARED_LIBS "Build LGPL Code as Shared Libraries (LGPL Code)" ON)
set (LEGO_LGPL_LIB_TYPE STATIC)
if (BUILD_LGPL_SHARED_LIBS)
  set (LEGO_LGPL_LIB_TYPE SHARED)
else ()
  message(STATUS "Building LGPL code as static library (affects license of the binary)")
endif ()

include(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(LEGO_BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
        "LEGO_BUILD_APPS" OFF)

# build the examples
set(LEGO_BUILD_EXAMPLES ON CACHE BOOL "Build lego examples")
if(LEGO_BUILD_EXAMPLES)
  message(STATUS "Compiling lego examples")
endif(LEGO_BUILD_EXAMPLES)

# gcc
if(CMAKE_COMPILER_IS_GNUCXX)
  option (BUILD_WITH_MARCH_NATIVE "Build with \"-march native\"" OFF)
  message(STATUS "Compiling with GCC")

  # settings for optimisation
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

  # switch off optimization for debug builds
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # Linux
  if(BUILD_WITH_MARCH_NATIVE AND NOT "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm" AND "${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
#    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
#    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  endif()
  # activate warnings
  set(lego_C_FLAGS "${lego_C_FLAGS} -Wall -W")
  set(lego_CXX_FLAGS "${lego_CXX_FLAGS} -Wall -W")
endif(CMAKE_COMPILER_IS_GNUCXX)

# c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# specifying compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${lego_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${lego_C_FLAGS}")

# find Eigen3
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
find_package(Eigen3 REQUIRED)
if (TARGET Eigen3::Eigen)
  set(LEGO_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

# find OpenMP
#find_package(OpenMP REQUIRED)
#if (OPENMP_FOUND)
#	message("OpenMP Found")
#	# cmake setting
#	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")
#endif ()

# Set up the top-level include directories
include_directories(${lego_SOURCE_DIR} ${PROJECT_BINARY_DIR})

# config.h
set(LEGO_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(LEGO_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
set(LEGO_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

configure_file(config.h.in "${PROJECT_BINARY_DIR}/lego/config.h")
install(FILES ${PROJECT_BINARY_DIR}/lego/config.h DESTINATION ${INCLUDES_DESTINATION}/lego)

# cmake configuration scripts
set(LEGO_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(LEGO_VERSION_CONFIG "${LEGO_GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(LEGO_PROJECT_CONFIG "${LEGO_GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(LEGO_TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(LEGO_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(LEGO_NAMESPACE "${PROJECT_NAME}::")
set(LEGO_VERSION 1.0.0)

include(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
        "${LEGO_VERSION_CONFIG}" VERSION ${LEGO_VERSION} COMPATIBILITY SameMajorVersion
)

configure_file("${lego_SOURCE_DIR}/cmake_modules/Config.cmake.in" "${LEGO_PROJECT_CONFIG}" @ONLY)

install(
    FILES "${LEGO_PROJECT_CONFIG}" "${LEGO_VERSION_CONFIG}"
    DESTINATION "${LEGO_CONFIG_INSTALL_DIR}")

install(
    EXPORT "${LEGO_TARGETS_EXPORT_NAME}"
    NAMESPACE "${LEGO_NAMESPACE}"
    DESTINATION "${LEGO_CONFIG_INSTALL_DIR}")

# subdirectories
add_subdirectory(lego)
