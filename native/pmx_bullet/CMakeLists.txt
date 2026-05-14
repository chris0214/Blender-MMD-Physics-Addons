cmake_minimum_required(VERSION 3.20)
project(pmx_bullet LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE                Release CACHE STRING "" FORCE)

set(BUILD_BULLET2_DEMOS             OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS             OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS                 OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS                OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS                    ON  CACHE BOOL "" FORCE)
set(USE_MSVC_RUNTIME_LIBRARY_DLL    ON  CACHE BOOL "" FORCE)
set(USE_GLUT                        OFF CACHE BOOL "" FORCE)
set(BUILD_EGL                       OFF CACHE BOOL "" FORCE)

add_subdirectory(bullet3 EXCLUDE_FROM_ALL)

add_library(pmx_bullet SHARED pmx_bullet_api.cpp)

target_include_directories(pmx_bullet PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/bullet3/src"
)

target_link_libraries(pmx_bullet PRIVATE
    BulletDynamics
    BulletCollision
    LinearMath
)

set_target_properties(pmx_bullet PROPERTIES
    PREFIX ""
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

if(MINGW)
    target_link_options(pmx_bullet PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
