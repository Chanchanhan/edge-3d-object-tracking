cmake_minimum_required(VERSION 3.1)
project(ROBOT)
enable_testing()
set( CMAKE_CXX_COMPILER "g++" )
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CmakeModules/)

##############################################################################
# C++14 support
##############################################################################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Ceres REQUIRED)

file(GLOB_RECURSE SOURCE_FILES  ${PROJECT_SOURCE_DIR}/src/*.c* ${PROJECT_SOURCE_DIR}/ThirdParty/*.c*)
file(GLOB_RECURSE HEADERS  ${PROJECT_SOURCE_DIR}/include/*.h* ${PROJECT_SOURCE_DIR}/ThirdParty/*.h*)

source_group("Include" FILES ${HEADERS})
source_group("Src" FILES ${SOURCE_FILES})


INCLUDE_DIRECTORIES(
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include/
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}

)


ADD_DEFINITIONS(/D_USE_MATH_DEFINES)

#LIST(APPEND LINK_LIBS ${PROJECT_SOURCE_DIR}/DBoW3001d.lib ${OpenCV_LIBS})
LIST(APPEND LINK_LIBS ceres ${GLOG_LIBRARY}  ${OpenCV_LIBS} ${Sophus_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
#TARGET_LINK_LIBRARIES(ROBOT ${LINK_LIBS})



#add test unit 1
add_executable(test-qqh main.cpp ${HEADERS} ${SOURCE_FILES} )
add_test(test-qqh test-qqh)
SET_TARGET_PROPERTIES(test-qqh PROPERTIES    RUNTIME_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/bin")
TARGET_LINK_LIBRARIES(test-qqh ${LINK_LIBS})
