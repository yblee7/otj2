cmake_minimum_required(VERSION 3.5)

project(ojt_homography VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PRODUCT_VERSION_MAJOR 0)
set(PRODUCT_VERSION_MINOR 0)
set(PRODUCT_VERSION_PATCH 0)
set(PRODUCT_VERSION_BUILD_NUMBER 1)

set(COMPANY_NAME "CLE Inc.")
set(FILE_DESCRIPTION "CLE's OJT Homography Rectifier")
set(INTERNAL_NAME "CLE's OJT Homography Rectifier")
set(ORIGINAL_FILENAME "ojt_homography")
set(LEGAL_COPYRIGHT "CLE Inc. All rights reserved.")
set(PRODUCT_NAME "CLE's OJT Homography Rectifier")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/version.rc.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.rc
    @ONLY)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED world)
find_package(Eigen3 REQUIRED)

set(PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mainwindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/include/image_viewer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/image_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/homography.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/homography.cpp
)

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ojt_homography MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ${RCCFILES}
        ${APP_ICON_RESOURCE_WINDOWS}
        ${CMAKE_CURRENT_BINARY_DIR}/version.rc)
else()
    if(ANDROID)
        add_library(ojt_image_viewer SHARED
            ${PROJECT_SOURCES}
            ${RCCFILES}
            ${APP_ICON_RESOURCE_WINDOWS}
        )
    else()
        add_executable(ojt_image_viewer
            ${PROJECT_SOURCES}
            ${RCCFILES}
            ${APP_ICON_RESOURCE_WINDOWS}
            ${CMAKE_CURRENT_BINARY_DIR}/version.rc
        )
    endif()
endif()

add_definitions(-DVERSION_MAJOR=${PRODUCT_VERSION_MAJOR})
add_definitions(-DVERSION_MINOR=${PRODUCT_VERSION_MINOR})
add_definitions(-DVERSION_PATCH=${PRODUCT_VERSION_PATCH})
add_definitions(-DVERSION_BUILD_NUMBER=${PRODUCT_VERSION_BUILD_NUMBER})

add_definitions(-DDEVEL)

if(MSVC)
    target_link_options(ojt_homography PRIVATE /SUBSYSTEM:CONSOLE)
endif()

target_link_libraries(ojt_homography PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${OpenCV_LIBRARIES} Eigen3::Eigen)

set_target_properties(ojt_homography PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER cle.vision
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ojt_homography)
endif()