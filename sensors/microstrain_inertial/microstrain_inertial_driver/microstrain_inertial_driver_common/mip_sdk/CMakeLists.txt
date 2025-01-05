
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(
    "MIP SDK"
    VERSION "0.0.01"
    DESCRIPTION "MicroStrain Communications Library for embedded systems"
    LANGUAGES C CXX
)

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(EXT_DIR "${CMAKE_CURRENT_LIST_DIR}/ext")
set(MIP_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake")

include_directories(
    "${SRC_DIR}"
)

if(WITH_INTERNAL)
    if(NOT DEFINED MIP_INTERNAL_DIR)
		set(MIP_INTERNAL_DIR "int" CACHE PATH "")
    endif()
	
	add_subdirectory("${MIP_INTERNAL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/mip-internal")
endif()

set(MIP_DIR "${SRC_DIR}/mip")

# Use Git to find the version
find_package(Git)
set(DEFAULT_MIP_GIT_VERSION "v0.0.0")
if(NOT GIT_FOUND)
  message(STATUS "Unable to find git, will build with unknown version")
  set(MIP_GIT_VERSION ${DEFAULT_MSCL_GIT_VERSION})
else()
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} describe --tags
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE MIP_GIT_VERSION_OUT
    ERROR_VARIABLE MIP_GIT_VERSION_ERR
    RESULT_VARIABLE MIP_GIT_VERSION_RET
  )
  if(NOT ${MIP_GIT_VERSION_RET} EQUAL 0)
    message(STATUS "Unable to determine version from Git, defaulting to version ${DEFAULT_MIP_GIT_VERSION}")
    set(MIP_GIT_VERSION ${DEFAULT_MIP_GIT_VERSION})
  else()
    set(MIP_GIT_VERSION ${MIP_GIT_VERSION_OUT})
    string(REGEX REPLACE "\n" "" MIP_GIT_VERSION "${MIP_GIT_VERSION}")
    message(STATUS "MIP SDK Version: ${MIP_GIT_VERSION}")
  endif()
endif()

# Massage the version number a little so we can use it in a couple places
string(REGEX REPLACE "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+).*" "\\1.\\2.\\3" MIP_GIT_VERSION_CLEAN ${MIP_GIT_VERSION})
string(REPLACE "." ";" MIP_GIT_VERSION_LIST ${MIP_GIT_VERSION_CLEAN})
list(LENGTH MIP_GIT_VERSION_LIST MIP_GIT_VERSION_LIST_LENGTH)
if (MIP_GIT_VERSION_LIST_LENGTH GREATER_EQUAL 3)
    list(GET MIP_GIT_VERSION_LIST 0 MIP_GIT_VERSION_MAJOR)
    list(GET MIP_GIT_VERSION_LIST 1 MIP_GIT_VERSION_MINOR)
    list(GET MIP_GIT_VERSION_LIST 2 MIP_GIT_VERSION_PATCH)
else()
    message(WARNING "MIP version cannot be parsed into a semantic version string.\nPlease run 'git fetch --tags' to get a properly tagged build")
    set(MIP_GIT_VERSION_CLEAN "0.0.0")
    set(MIP_GIT_VERSION_MAJOR 0)
    set(MIP_GIT_VERSION_MINOR 0)
    set(MIP_GIT_VERSION_PATCH 0)
endif()

# Generate the version header file
set(VERSION_IN_FILE "${MIP_CMAKE_DIR}/mip_version.h.in")
set(VERSION_OUT_FILE "${MIP_DIR}/mip_version.h")
configure_file(${VERSION_IN_FILE} ${VERSION_OUT_FILE})

#
# Build options
#

option(MIP_USE_SERIAL "Build serial connection support into the library and examples" ON)
option(MIP_USE_TCP    "Build TCP connection support into the library and exampels" ON)
option(MIP_USE_EXTRAS "Build extra support into the library including some things that might work at a higher level and use dynamically allocated memory" ON)
option(MIP_ENABLE_DIAGNOSTICS "Enable various diagnostic counters in the mip library for debugging." OFF)

option(MIP_ENABLE_LOGGING "Build with logging functions enabled" ON)
set(MIP_LOGGING_MAX_LEVEL "" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any log level logged at a higher level than this will result in a noop regardless of runtime configuration.")

set(MIP_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")

option(BUILD_PACKAGE "Whether to build a package from the resulting binaries" OFF)

#
# Utils
#

set(UTILS_DIR "${MIP_DIR}/utils")

set(UTILS_SOURCES
    "${UTILS_DIR}/byte_ring.c"
    "${UTILS_DIR}/byte_ring.h"
    "${UTILS_DIR}/serialization.c"
    "${UTILS_DIR}/serialization.h"
)

#
# MIP Control
#

set(MIP_SOURCES
    "${VERSION_OUT_FILE}"
    "${MIP_DIR}/mip_cmdqueue.c"
    "${MIP_DIR}/mip_cmdqueue.h"
    "${MIP_DIR}/mip_dispatch.c"
    "${MIP_DIR}/mip_dispatch.h"
    "${MIP_DIR}/mip_field.c"
    "${MIP_DIR}/mip_field.h"
    "${MIP_DIR}/mip_offsets.h"
    "${MIP_DIR}/mip_packet.c"
    "${MIP_DIR}/mip_packet.h"
    "${MIP_DIR}/mip_parser.c"
    "${MIP_DIR}/mip_parser.h"
    "${MIP_DIR}/mip_result.c"
    "${MIP_DIR}/mip_result.h"
    "${MIP_DIR}/mip_types.h"
    "${MIP_DIR}/definitions/common.c"
    "${MIP_DIR}/definitions/common.h"
    "${MIP_DIR}/definitions/descriptors.c"
    "${MIP_DIR}/definitions/descriptors.h"
    "${MIP_DIR}/mip.hpp"
    "${MIP_DIR}/mip_all.h"
    "${MIP_DIR}/mip_all.hpp"
)

set(MIPDEV_SOURCES
    "${MIP_DIR}/mip_interface.c"
    "${MIP_DIR}/mip_interface.h"
    "${MIP_DIR}/mip_device.cpp"
    "${MIP_DIR}/mip_device.hpp"
)

set(MIPDEF_SOURCES
    "${MIP_DIR}/definitions/commands_3dm.c"
    "${MIP_DIR}/definitions/commands_3dm.h"
    "${MIP_DIR}/definitions/commands_base.c"
    "${MIP_DIR}/definitions/commands_base.h"
    "${MIP_DIR}/definitions/commands_filter.c"
    "${MIP_DIR}/definitions/commands_filter.h"
    "${MIP_DIR}/definitions/commands_gnss.c"
    "${MIP_DIR}/definitions/commands_gnss.h"
    "${MIP_DIR}/definitions/commands_rtk.c"
    "${MIP_DIR}/definitions/commands_rtk.h"
    "${MIP_DIR}/definitions/commands_aiding.c"
    "${MIP_DIR}/definitions/commands_aiding.h"
    "${MIP_DIR}/definitions/commands_system.c"
    "${MIP_DIR}/definitions/commands_system.h"
    "${MIP_DIR}/definitions/data_filter.c"
    "${MIP_DIR}/definitions/data_filter.h"
    "${MIP_DIR}/definitions/data_gnss.c"
    "${MIP_DIR}/definitions/data_gnss.h"
    "${MIP_DIR}/definitions/data_sensor.c"
    "${MIP_DIR}/definitions/data_sensor.h"
    "${MIP_DIR}/definitions/data_shared.c"
    "${MIP_DIR}/definitions/data_shared.h"
    "${MIP_DIR}/definitions/data_system.c"
    "${MIP_DIR}/definitions/data_system.h"
    ${INTDEF_SOURCES}
)

string(REGEX REPLACE "\.h(;|$)" ".hpp\\1" MIPDEF_HPP_SOURCES "${MIPDEF_SOURCES}")
string(REGEX REPLACE "\.c(;|$)" ".cpp\\1" MIPDEF_CPP_SOURCES "${MIPDEF_HPP_SOURCES}")

if(MIP_USE_SERIAL)
    list(APPEND UTILS_SOURCES
        "${UTILS_DIR}/serial_port.c"
        "${UTILS_DIR}/serial_port.h"
    )
    list(APPEND MIP_INTERFACE_SOURCES
        "${MIP_DIR}/platform/serial_connection.hpp"
        "${MIP_DIR}/platform/serial_connection.cpp"
    )
endif()
if(MIP_USE_TCP)
    list(APPEND UTILS_SOURCES
        "${UTILS_DIR}/tcp_socket.c"
        "${UTILS_DIR}/tcp_socket.h"
    )
    list(APPEND MIP_INTERFACE_SOURCES
        "${MIP_DIR}/platform/tcp_connection.hpp"
        "${MIP_DIR}/platform/tcp_connection.cpp"
    )
endif()
if(MIP_USE_EXTRAS)
    list(APPEND MIP_DEFINES "MIP_USE_EXTRAS")
    set(MIP_EXTRAS_DIR "${MIP_DIR}/extras")
    set(MIP_EXTRA_SOURCES
        "${MIP_EXTRAS_DIR}/composite_result.hpp"
        "${MIP_EXTRAS_DIR}/descriptor_id.hpp"
        "${MIP_EXTRAS_DIR}/device_models.h"
        "${MIP_EXTRAS_DIR}/device_models.c"
        "${MIP_EXTRAS_DIR}/index.hpp"
        "${MIP_EXTRAS_DIR}/recording_connection.hpp"
        "${MIP_EXTRAS_DIR}/recording_connection.cpp"
        "${MIP_EXTRAS_DIR}/scope_helper.hpp"
        "${MIP_EXTRAS_DIR}/scope_helper.cpp"
        "${MIP_EXTRAS_DIR}/version.hpp"
        "${MIP_EXTRAS_DIR}/version.cpp"
    )
endif()

# Logging is a little weird since we need to install the header no matter what
list(APPEND MIP_SOURCES "${MIP_DIR}/mip_logging.h")
if(MIP_ENABLE_LOGGING)
    list(APPEND MIP_SOURCES "${MIP_DIR}/mip_logging.c")
    list(APPEND MIP_DEFINES "MIP_ENABLE_LOGGING")
endif()

set(ALL_MIP_SOURCES
    ${MIPDEF_SOURCES}
    ${MIPDEF_CPP_SOURCES}
    ${MIPDEV_SOURCES}
    ${MIP_SOURCES}
    ${UTILS_SOURCES}
    ${MIP_CPP_HEADERS}
    ${MIP_INTERFACE_SOURCES}
    ${MIP_EXTRA_SOURCES}
)

option(MIP_DISABLE_CPP "Excludes all C++ files from the project." OFF)
if(MIP_DISABLE_CPP)
    list(FILTER ALL_MIP_SOURCES EXCLUDE REGEX "[c|h]pp$")
endif()

add_library(mip ${ALL_MIP_SOURCES})

#
# Preprocessor definitions
#

if(${MIP_LOGGING_MAX_LEVEL})
    list(APPEND MIP_DEFINES "MIP_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}")
endif()

if(${MIP_TIMESTAMP_TYPE})
    list(APPEND MIP_DEFINES "MIP_TIMESTAMP_TYPE=${MIP_TIMESTAMP_TYPE}")
endif()

if(${MIP_ENABLE_DIAGNOSTICS})
    list(APPEND MIP_DEFINES "MIP_ENABLE_DIAGNOSTICS")
endif()

# Disable windows defined min/max
# Set Windows header version (0x0501 is _WIN32_WINNT_WINXP, required for TCP)
if(WIN32)
  list(APPEND MIP_DEFINES "NOMINMAX=1" "_WIN32_WINNT=0x0501")
endif()

if(MSVC)
  target_compile_options(mip PRIVATE /W4)
else()
  target_compile_options(mip PRIVATE -Wall -Wextra)
endif()

target_compile_definitions(mip PUBLIC "${MIP_DEFINES}")

#
# Libraries
#
if(WIN32 AND MIP_USE_TCP)
    target_link_libraries(mip PUBLIC "ws2_32")
endif()

#
# TESTING
#

# CTest defines this option to ON by default, so override it to OFF here.
option(BUILD_TESTING "Build the testing tree." OFF)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory("test")
endif()

#
# EXAMPLES
#

option(BUILD_EXAMPLES "Builds the example programs." ON)

if(BUILD_EXAMPLES)
    add_subdirectory("examples")
endif()

#
# DOCUMENTATION
#

find_package(Doxygen)
option(BUILD_DOCUMENTATION       "Build the documentation." OFF)
option(BUILD_DOCUMENTATION_FULL  "Build the full (internal) documentation." OFF)
option(BUILD_DOCUMENTATION_QUIET "Suppress doxygen standard output." ON)

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is required to build documentation.")
    endif()

    set(DOXYGEN_PROJECT_NUMBER "${MIP_GIT_VERSION}")

    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/documentation")
    set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_LIST_DIR}/docs")

    set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)

    if(NOT MIP_DISABLE_CPP)
        set(DOXYGEN_PREDEFINED "__cplusplus")
    endif()

    set(DOXYGEN_EXTRACT_ALL YES)

    if(BUILD_DOCUMENTATION_FULL)
        set(DOXYGEN_INTERNAL_DOCS YES)
        # set(DOXYGEN_WARN_AS_ERROR YES)
    else()
        set(DOXYGEN_HIDE_UNDOC_MEMBERS YES)
        set(DOXYGEN_HIDE_UNDOC_CLASSES YES)
    endif()

    if(BUILD_DOCUMENTATION_QUIET)
        set(DOXYGEN_QUIET YES)
    endif()

    doxygen_add_docs(docs
        "${SRC_DIR}" "${CMAKE_CURRENT_LIST_DIR}/docs"
        COMMENT "Generating documentation."
    )

    # Add a target to enable users to zip up the docs
    add_custom_target(package_docs
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/documentation/html
        DEPENDS docs
        COMMAND ${CMAKE_COMMAND} -E tar "cf" "${CMAKE_CURRENT_BINARY_DIR}/mipsdk_${MIP_GIT_VERSION}_Documentation.zip" --format=zip "."
    )
endif(BUILD_DOCUMENTATION)

#
# Packaging
#

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/mip")

set(EXPORT_TARGETS mip)
configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/mip-config.cmake.in"
    "${CMAKE_BINARY_DIR}/mip-config.cmake"
    INSTALL_DESTINATION "${CONFIG_EXPORT_DIR}"
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CONFIG_EXPORT_DIR
)
unset(EXPORT_TARGETS)

write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/mip-config-version.cmake"
    COMPATIBILITY AnyNewerVersion
)

#
# Installation
#

# Only install headers that we build the source files for
set(ALL_MIP_HEADERS "${ALL_MIP_SOURCES}")
list(FILTER ALL_MIP_HEADERS INCLUDE REGEX "^.*\.(h|hpp)$")
foreach(MIP_HEADER ${ALL_MIP_HEADERS})
    string(REPLACE "${SRC_DIR}/" "" MIP_HEADER_RELATIVE "${MIP_HEADER}")
    if(INT_DIR)
        string(REPLACE "${INT_DIR}/" "" MIP_HEADER_RELATIVE "${MIP_HEADER_RELATIVE}")
    endif()
    set(MIP_HEADER_DESTINATION_FULL "${CMAKE_INSTALL_INCLUDEDIR}/${MIP_HEADER_RELATIVE}")
    get_filename_component(MIP_HEADER_DESTINATION "${MIP_HEADER_DESTINATION_FULL}" DIRECTORY)
    install(
        FILES "${MIP_HEADER}"
        DESTINATION "${MIP_HEADER_DESTINATION}"
        COMPONENT mip
    )
endforeach()

install(TARGETS
    mip
    EXPORT mip-targets
    ARCHIVE
    COMPONENT mip
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/"
)

install(EXPORT
    mip-targets
    COMPONENT mip
    DESTINATION "${CONFIG_EXPORT_DIR}"
)

install(FILES
    "${CMAKE_BINARY_DIR}/mip-config.cmake"
    "${CMAKE_BINARY_DIR}/mip-config-version.cmake"
    COMPONENT mip
    DESTINATION "${CONFIG_EXPORT_DIR}"
)

# Try to determine what architecture we are building for based on the compiler output
if(MSVC)
    # Detect if this is a x64 or x86 build
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(MIP_ARCH "x64")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(MIP_ARCH "x86")
    endif()
elseif(UNIX)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # The dumpmachine command from gcc should contain information on the architecture
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "${CMAKE_CXX_COMPILER} -dumpmachine"
            OUTPUT_VARIABLE GCC_ARCHITECTURE_OUT
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            ERROR_VARIABLE GCC_ARCHITECTURE_ERR
            RESULT_VARIABLE GCC_ARCHITECTURE_RET
        )

        # Convert the GCC architecture to the format that we use
        if("${GCC_ARCHITECTURE_OUT}" MATCHES ".*x86_64.*")
            set(MIP_ARCH "amd64")
        elseif("${GCC_ARCHITECTURE_OUT}" MATCHES ".*aarch64.*")
            set(MIP_ARCH "arm64")
        elseif("${GCC_ARCHITECTURE_OUT}" MATCHES ".*arm.*")
            set(MIP_ARCH "armhf")
        else()
            message(STATUS "Unrecognized GCC architecture ${GCC_ARCHITECTURE_OUT}. Using CMAKE_SYSTEM_PROCESSOR for architecture")
        endif()
    endif()
endif()
if(NOT DEFINED MIP_ARCH)
    message(STATUS "Defaulting MIP_ARCH to ${CMAKE_SYSTEM_PROCESSOR}")
    set(MIP_ARCH ${CMAKE_SYSTEM_PROCESSOR})
endif()

# If we were asked to package, find the generators we can use
if(BUILD_PACKAGE)
    set(FOUND_CPACK_GENERATORS "")
    set(7Z_ROOT "" CACHE STRING "Location of the 7zip executable")
    set(DPKG_ROOT "" CACHE STRING "Location of the dpkg executable")
    set(RPMBUILD_ROOT "" CACHE STRING "Location of the rpmbuild executable")
    find_program(7Z_EXECUTABLE
        NAMES 7z
        PATHS ${7Z_ROOT}
        DOC "7zip command line client"
    )
    if(NOT ${7Z_EXECUTABLE} STREQUAL "7Z_EXECUTABLE-NOTFOUND")
        list(APPEND FOUND_CPACK_GENERATORS "7Z")
    endif()
    find_program(DPKG_EXECUTABLE
        NAMES dpkg
        PATHS ${DPKG_ROOT}
        DOC "dpkg command line client"
    )
    if(NOT ${DPKG_EXECUTABLE} STREQUAL "DPKG_EXECUTABLE-NOTFOUND")
        list(APPEND FOUND_CPACK_GENERATORS "DEB")
    endif()
    find_program(RPMBUILD_EXECUTABLE
        NAMES rpmbuild
        PATHS ${RPMBUILD_ROOT}
        DOC "rpmbuild command line client"
    )
    if(NOT ${RPMBUILD_EXECUTABLE} STREQUAL "RPMBUILD_EXECUTABLE-NOTFOUND")
        list(APPEND FOUND_CPACK_GENERATORS "RPM")
    endif()
    if(NOT FOUND_CPACK_GENERATORS)
        message(FATAL_ERROR "Unable to find a suitable package generator, but we were requested to build a package.")
    endif()
endif()

# If we were asked to build packages, include CPack and set up packaging
if(BUILD_PACKAGE)
    # NOTE: CPack requires all these variables to be set before importing the module. Do not move them after the include(CPack) line
    set(CPACK_GENERATOR "${FOUND_CPACK_GENERATORS}")
    set(CPACK_PACKAGE_VENDOR "Parker Hanifin")
    set(CPACK_PACKAGE_CONTACT "Rob Fisher <rob.fisher@parker.com>")
    set(CPACK_COMPONENTS_ALL "mip")

    set(CPACK_PACKAGE_VERSION ${MIP_GIT_VERSION_CLEAN})

    # Shared configuration
    set(MIP_FILE_NAME_PREFIX "mipsdk_${MIP_GIT_VERSION}_${MIP_ARCH}")

    # DEB specific configuration
    # Build different deb packages for each target
    set(CPACK_DEBIAN_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}.deb")
    set(CPACK_DEB_COMPONENT_INSTALL ON)
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

    # RPM specific configuration
    # Build different RPM packages for each target
    set(CPACK_RPM_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}.rpm")
    set(CPACK_RPM_COMPONENT_INSTALL ON)
    set(CPACK_RPM_PACKAGE_AUTOREQ ON)

    # Zip specific configuration
    # Build different zip packages for each target
    if(MSVC)
        set(CPACK_ARCHIVE_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}_MSVC_v${MSVC_TOOLSET_VERSION}")
    elseif(APPLE)
        set(CPACK_ARCHIVE_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}_OSX")
    elseif(UNIX)
        set(CPACK_ARCHIVE_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}_Linux")
    else()
        set(CPACK_ARCHIVE_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}")
    endif()
    set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

    # Finally include cpack which should have taken all of the previous variables into consideration
    include(CPack)

    cpack_add_component(mip
        DESCRIPTION "MIP SDK static library and header files"
        GROUP mip
    )
endif()
