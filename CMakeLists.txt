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

include(FetchContent)

set(SFML_VERSION "2.5.1")
FetchContent_Declare(
    sfml
    GIT_REPOSITORY "https://github.com/SFML/SFML.git"
    GIT_TAG        "${SFML_VERSION}"
)

FetchContent_GetProperties(sfml)
if(NOT sfml_POPULATED)
    FetchContent_Populate(sfml)
    add_subdirectory(${sfml_SOURCE_DIR} ${sfml_BINARY_DIR})
endif()

# Compile executable
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

# Link executable to required SFML libraries
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-audio
        sfml-graphics
        sfml-system
        sfml-window
        )

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/saves)
