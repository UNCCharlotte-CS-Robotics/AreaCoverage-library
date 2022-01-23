cmake_minimum_required(VERSION 3.12)

project(aclibrary VERSION 1.0)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)
# include(InstallRequiredSystemLibraries)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
# find_package(CGAL REQUIRED)
find_package(lclibrary REQUIRED)

message(STATUS "CGAL_MAJOR_VERSION: ${CGAL_MAJOR_VERSION}")
message(STATUS "CGAL_MINOR_VERSION: ${CGAL_MINOR_VERSION}")

set(ACLIBRARY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(ACLIBRARY_EXTERNAL_JSON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/json/include/)
set(ACLIBRARY_EXTERNAL_PCP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/pcp/include/)

option(ACLIBRARY_BUILD_SHARED_LIBS "Build aclibrary shared library" ${BUILD_SHARED_LIBS})

set(build-shared $<BOOL:${ACLIBRARY_BUILD_SHARED_LIBS}>)
if (ACLIBRARY_BUILD_SHARED_LIBS)
	set(aclibrary-type SHARED)
	set(aclibrary-label-postfix "shared")
else()
	set(aclibrary-type STATIC)
	set(aclibrary-label-postfix "static")
endif()


set(ACLIBRARY_EXTERNAL_PCP_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/pcp/)
set(pcp-src-pattern "${ACLIBRARY_EXTERNAL_PCP_SOURCE_DIR}/src/*.cc")
file(GLOB pcp-sources ${pcp-src-pattern})

set(aclibrary-src-files
	env.cc
	env_io.cc
	env_split.cc
	env_decomposition.cc
	polygon_utilities.cc)

list(TRANSFORM aclibrary-src-files PREPEND "${PROJECT_SOURCE_DIR}/src/")

add_library(aclibrary ${aclibrary-type} "")

target_compile_options(aclibrary PRIVATE
	-O2 -fPIC
	-Wall -pedantic)
# -Wextra -Wshadow -Weffc++ -Wno-long-long
# -pedantic -pedantic-errors)

target_include_directories(aclibrary PUBLIC
	$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
	$<BUILD_INTERFACE:${ACLIBRARY_EXTERNAL_JSON_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ACLIBRARY_EXTERNAL_PCP_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ACLIBRARY_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(aclibrary PRIVATE $<BUILD_INTERFACE:${LCLIBRARY_INCLUDE_DIR}>)
target_include_directories(aclibrary PRIVATE ${CGAL_INCLUDE_DIRS})

# target_link_libraries(aclibrary PRIVATE pcplibrary)
target_link_libraries(aclibrary PRIVATE lclibrary stdc++fs m yaml-cpp mpfr gmp gmpxx)

target_sources(aclibrary PRIVATE ${aclibrary-src-files} ${pcp-sources})

set_target_properties(aclibrary PROPERTIES
	VERSION "${PROJECT_VERSION}"
	SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
	PROJECT_LABEL "aclibrary ${aclibrary-label-postfix}")

# message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${LCLIBRARY_INCLUDE_DIR}")
install(TARGETS aclibrary
	EXPORT aclibrary-targets
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY
	${PROJECT_SOURCE_DIR}/include/
	${ACLIBRARY_EXTERNAL_JSON_INCLUDE_DIR}
	${ACLIBRARY_EXTERNAL_PCP_INCLUDE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES "${PROJECT_BINARY_DIR}/aclibrary_config.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config" DESTINATION ${CMAKE_INSTALL_BINDIR}/)
install(TARGETS aclibrary LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT aclibrary-targets
	FILE aclibrary-targets.cmake
	DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/aclibrary")

include(CMakePackageConfigHelpers)

# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/aclibrary-config.cmake.in
	"${CMAKE_CURRENT_BINARY_DIR}/aclibrary-config.cmake"
	INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/aclibrary")

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/aclibrary-config-version.cmake"
	VERSION "${aclibrary_VERSION_MAJOR}.${aclibrary_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/aclibrary-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/aclibrary-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aclibrary)

export(EXPORT aclibrary-targets
	FILE "${CMAKE_CURRENT_BINARY_DIR}/aclibrary-targets.cmake")

add_executable(mac_point_robot ${PROJECT_SOURCE_DIR}/main/mac_point_robot.cc)
install(TARGETS mac_point_robot DESTINATION ${CMAKE_INSTALL_BINDIR}/)
target_include_directories(mac_point_robot PRIVATE ${CGAL_INCLUDE_DIRS})
target_link_libraries(mac_point_robot PRIVATE lclibrary yaml-cpp aclibrary)
target_compile_options(mac_point_robot PRIVATE -O2 -Wall -pedantic)

add_executable(mac_custom_robot ${PROJECT_SOURCE_DIR}/main/mac_custom_robot.cc)
install(TARGETS mac_custom_robot DESTINATION ${CMAKE_INSTALL_BINDIR}/)
target_include_directories(mac_custom_robot PRIVATE ${CGAL_INCLUDE_DIRS})
target_link_libraries(mac_custom_robot PRIVATE lclibrary yaml-cpp aclibrary)
target_compile_options(mac_custom_robot PRIVATE -O2 -Wall -pedantic)

configure_file(aclibrary_config.h.in aclibrary_config.h)

