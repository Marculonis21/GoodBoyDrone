cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Set project name and version
project(Drone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

file(GLOB SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp)

file(GLOB HEADER_FILES
    ${CMAKE_SOURCE_DIR}/src/*.hpp)

# Find SFML shared libraries
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# Compile executable
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

# Link executable to required SFML libraries
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-audio
        sfml-graphics
        sfml-system
        sfml-window
        )

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/saves)
