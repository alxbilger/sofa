cmake_minimum_required(VERSION 3.12)

project(SofaGraphComponent_test)

set(SOURCE_FILES ../../empty.cpp)

list(APPEND SOURCE_FILES
SceneChecker_test.cpp
)

sofa_find_package(SofaGraphComponent REQUIRED)

#add_definitions("-DSOFAGENERALTOPOLOGY_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/scenes\"")
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Sofa.Testing SofaBaseUtils SofaBaseMechanics SofaMeshCollision SofaGraphComponent)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
