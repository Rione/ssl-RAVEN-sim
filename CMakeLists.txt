cmake_minimum_required(VERSION 3.16)
project(m2-Sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC OFF)

# ===============================
# Platform settings
# ===============================
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    # Ask Homebrew where it is instead of hardcoding a prefix: the path differs
    # between Apple Silicon (/opt/homebrew) and Intel (/usr/local).
    #
    # Nothing here is version-pinned on purpose. This block used to point at
    # protobuf@21 and boost@1.85 with absolute Cellar paths, and both formulae
    # have since been disabled in Homebrew (protobuf@21 on 2026-01-08,
    # boost@1.85 on 2026-04-05), so those paths no longer exist on a new
    # machine. Install the unversioned formulae instead:
    #   brew install qt cmake boost protobuf
    execute_process(
        COMMAND brew --prefix
        OUTPUT_VARIABLE HOMEBREW_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(HOMEBREW_PREFIX)
        list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}" "${HOMEBREW_PREFIX}/opt/qt")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Not verified. Relies on Qt, Boost and Protobuf being discoverable from
    # the system prefixes.
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Windows (MinGW) settings
    # Dependencies installed via vcpkg:
    #   vcpkg install boost-asio:x64-mingw-dynamic protobuf:x64-mingw-dynamic
    # Run cmake with:
    #   -DCMAKE_TOOLCHAIN_FILE=C:/ws/vcpkg/scripts/buildsystems/vcpkg.cmake
    #   -DVCPKG_TARGET_TRIPLET=x64-mingw-dynamic
    #
    # No Qt version is hardcoded here. Give cmake an explicit location with
    # -DCMAKE_PREFIX_PATH=C:/Qt/<version>/mingw_64 (build-windows.ps1 does this)
    # or the QTDIR environment variable. Anything found below is only appended,
    # so an explicit path always takes precedence; it exists so that a plain
    # "cmake -S . -B build" still finds Qt.
    if(DEFINED ENV{QTDIR})
        list(APPEND CMAKE_PREFIX_PATH "$ENV{QTDIR}")
    else()
        file(GLOB QT_CANDIDATES "C:/Qt/6.*/mingw_64")
        if(QT_CANDIDATES)
            list(SORT QT_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
            list(GET QT_CANDIDATES 0 QT_DETECTED_DIR)
            list(APPEND CMAKE_PREFIX_PATH "${QT_DETECTED_DIR}")
            message(STATUS "Qt fallback location: ${QT_DETECTED_DIR}")
        endif()
    endif()
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

# ===============================
# Qt
# ===============================
find_package(Qt6 REQUIRED COMPONENTS
    Core Gui Widgets Network
    Quick Quick3D Quick3DPhysics Qml
)

# ===============================
# Boost / Protobuf
# ===============================
find_package(Boost REQUIRED)

# protobuf_MODULE_COMPATIBLE makes protobuf's own config package define
# protobuf_generate_cpp() and the Protobuf_* variables the find module used to
# set, so the rest of this file stays the same either way.
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(protobuf CONFIG QUIET)
if(NOT protobuf_FOUND)
    # Older installs ship no config package; fall back to CMake's find module.
    find_package(Protobuf REQUIRED)
endif()

# Link the imported target rather than ${Protobuf_LIBRARIES}: that variable
# holds a bare library path, which drops the Abseil dependency that protobuf 22
# and newer require and shows up as undefined absl:: symbols at link time.
if(TARGET protobuf::libprotobuf)
    set(PROTOBUF_LINK_LIBRARIES protobuf::libprotobuf)
else()
    set(PROTOBUF_LINK_LIBRARIES ${Protobuf_LIBRARIES})
endif()

# ===============================
# Source layout
# ===============================
set(SRC_DIR        ${CMAKE_SOURCE_DIR}/src)
set(MODEL_DIR      ${SRC_DIR}/models)
set(NET_DIR        ${SRC_DIR}/networks)
set(UTIL_DIR       ${SRC_DIR}/utils)

set(PROTO_DIR      ${CMAKE_SOURCE_DIR}/proto)
set(PROTO_SRC_DIR  ${PROTO_DIR}/pb_src)
set(PROTO_GEN_DIR  ${PROTO_DIR}/pb_gen)

file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

file(GLOB SRC_FILES
    ${SRC_DIR}/*.cpp
    ${MODEL_DIR}/*.cpp
    ${NET_DIR}/*.cpp
    ${UTIL_DIR}/*.cpp
)

file(GLOB HEADER_FILES
    ${SRC_DIR}/*.h
    ${MODEL_DIR}/*.h
    ${NET_DIR}/*.h
    ${UTIL_DIR}/*.h
)

file(GLOB PROTO_FILES "${PROTO_SRC_DIR}/*.proto")

protobuf_generate_cpp(
    PROTO_SRCS
    PROTO_HDRS
    ${PROTO_FILES}
    IMPORT_DIRS ${PROTO_SRC_DIR}
    PROTOC_OUT_DIR ${PROTO_GEN_DIR}
)

# ===============================
# Executable
# ===============================
add_executable(m2-Sim
    main.cpp
    ${SRC_FILES}
    ${HEADER_FILES}
    ${PROTO_SRCS}
)

# ===============================
# Include / Link
# ===============================
target_include_directories(m2-Sim PRIVATE
    ${SRC_DIR}
    ${PROTO_GEN_DIR}
    ${Boost_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(m2-Sim PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::Quick
    Qt6::Quick3D
    Qt6::Quick3DPhysics
    Qt6::Qml
    ${PROTOBUF_LINK_LIBRARIES}
)
if(WIN32)
    target_link_libraries(m2-Sim PRIVATE ws2_32 mswsock)
else()
    target_link_libraries(m2-Sim PRIVATE pthread)
endif()

# ===============================
# Output
# ===============================
set_target_properties(m2-Sim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ===============================
# Run helper
# ===============================
add_custom_target(run
    COMMAND ${CMAKE_BINARY_DIR}/bin/m2-Sim
    DEPENDS m2-Sim
)
