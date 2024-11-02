cmake_minimum_required(VERSION 3.27)
project(ParticleSystem)

set(CMAKE_CXX_STANDARD 17)

list(PREPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(eigen)
include(igl)
include(geogram)

add_subdirectory(src)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE ParticleSystem)