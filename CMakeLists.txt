cmake_minimum_required(VERSION 3.1.0)
project(testICP)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)


set(CCCoreLib_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/third/include)
set(CCCoreLib_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/third/lib)
add_compile_definitions(CC_CORE_LIB_USES_FLOAT)
message(INFO "CCCoreLib include path:" ${CCCoreLib_INCLUDE_DIRS})
message(INFO "CCCoreLib link path:" ${CCCoreLib_LIBRARY_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${CCCoreLib_INCLUDE_DIRS}
)

link_directories(${CCCoreLib_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} tests/test.cpp tests/DistanceComputationTools.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  CCCoreLib
)
