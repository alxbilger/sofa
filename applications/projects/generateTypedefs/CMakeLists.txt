cmake_minimum_required(VERSION 3.22)
project(generateTypedefs)

find_package(SofaGeneral)
find_package(SofaMisc)

add_executable(${PROJECT_NAME} Main.cpp)
target_link_libraries(${PROJECT_NAME} SofaGeneral SofaMisc)
