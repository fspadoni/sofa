cmake_minimum_required(VERSION 3.12)
project(Geomagic VERSION 0.1)

find_package(SofaHaptics REQUIRED)
find_package(SofaOpenglVisual REQUIRED)
find_package(SofaUserInteraction REQUIRED)
find_package(SofaLoader REQUIRED)

find_package(OpenHaptics REQUIRED)

if( NOT OPENHAPTICS_FOUND )
    message( SEND_ERROR "OpenHaptics library not found, GeomagicDriver cannot compile")
endif()

set(GEOMAGIC_SRC_DIR src/Geomagic)

set(HEADER_FILES
    ${GEOMAGIC_SRC_DIR}/config.h.in
    ${GEOMAGIC_SRC_DIR}/GeomagicDriver.h
    ${GEOMAGIC_SRC_DIR}/GeomagicEmulator.h
    ${GEOMAGIC_SRC_DIR}/GeomagicVisualModel.h
)

set(SOURCE_FILES	  
    ${GEOMAGIC_SRC_DIR}/GeomagicDriver.cpp
    ${GEOMAGIC_SRC_DIR}/GeomagicEmulator.cpp
    ${GEOMAGIC_SRC_DIR}/GeomagicVisualModel.cpp
    ${GEOMAGIC_SRC_DIR}/initPlugin.cpp
)

set(README_FILES README.txt)


# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaHelper SofaUserInteraction SofaHaptics SofaLoader SofaOpenglVisual ${OPENHAPTICS_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${OPENHAPTICS_INCLUDE_DIR}>")

## Install rules for the library; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
