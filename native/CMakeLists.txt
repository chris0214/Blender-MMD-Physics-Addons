cmake_minimum_required(VERSION 3.20)
project(pmx_bullet LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED BULLET_ROOT)
    message(FATAL_ERROR "Set BULLET_ROOT to a Bullet 2.82 r2704 source or install directory")
endif()

add_library(pmx_bullet SHARED pmx_bullet_api.cpp)

target_include_directories(pmx_bullet PRIVATE
    "${BULLET_ROOT}/src"
    "${BULLET_ROOT}/include"
    "${BULLET_ROOT}/include/bullet"
)

find_library(BULLET_DYNAMICS BulletDynamics PATHS "${BULLET_ROOT}/lib" "${BULLET_ROOT}/build/lib")
find_library(BULLET_COLLISION BulletCollision PATHS "${BULLET_ROOT}/lib" "${BULLET_ROOT}/build/lib")
find_library(LINEAR_MATH LinearMath PATHS "${BULLET_ROOT}/lib" "${BULLET_ROOT}/build/lib")

if(NOT BULLET_DYNAMICS OR NOT BULLET_COLLISION OR NOT LINEAR_MATH)
    message(FATAL_ERROR "Bullet libraries not found under BULLET_ROOT")
endif()

target_link_libraries(pmx_bullet PRIVATE
    "${BULLET_DYNAMICS}"
    "${BULLET_COLLISION}"
    "${LINEAR_MATH}"
)

if(MINGW)
    target_link_options(pmx_bullet PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
