cmake_minimum_required(VERSION 2.6)
project(epnp)

set(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(Sophus_INCLUDE_DIR "/usr/local/sophus-template/include")
# message(${Sophus_INCLUDE_DIR})
# OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Sophus REQUIRED )
FIND_PACKAGE(PCL REQUIRED)
#find_package(G2O REQUIRED)
find_package( CSparse)
# SET(G2O_DIR /usr/local/include/g2o )
# FIND_PACKAGE(G2O REQUIRED)

# FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )


# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )




# Eigen
#message(${Sophus_INCLUDE_DIR})
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    #/usr/local/include/
    ${G2O_INCLUDE_DIRS} 
    ${CSPARSE_INCLUDE_DIR} 
)


ADD_LIBRARY( slambase ./src/slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBS}
    ${PCL_LIBRARIES} 
    ${G2O_LIBRARIES} 
    # /usr/local/lib
    ${G2O_LIB} 
    ${CSPARSE_LIB}
    )


# add_executable(epnp main.cpp)
# target_link_libraries(epnp 
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
# ${Sophus_LIBS}
# )

# add_executable(pointcloud ./src/generatePointCloud.cpp)
# target_link_libraries(pointcloud 
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
# ${Sophus_LIBS}
# ${PCL_LIBRARIES} 
# )

# ADD_EXECUTABLE( detectFeatures ./src/detectFeatures.cpp)
# TARGET_LINK_LIBRARIES( detectFeatures 
#     slambase
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES} )



# ADD_EXECUTABLE( visualOdometry ./src/visualOdometry.cpp)
# TARGET_LINK_LIBRARIES( visualOdometry 
#     slambase
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES}
#     #${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#     ${CSPARSE_LIB} )


# ADD_EXECUTABLE( slamend ./src/slamEnd.cpp)
# TARGET_LINK_LIBRARIES( slamend 
#     slambase
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES}
#     ${G2O_LIB} 
#     ${CSPARSE_LIB} 
#     g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension 
#     ${CSPARSE_LIBRARY}
#     )


ADD_EXECUTABLE( slam ./src/slam.cpp)
TARGET_LINK_LIBRARIES( slam 
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${G2O_LIB} 
    ${CSPARSE_LIB} 
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension 
    ${CSPARSE_LIBRARY}
    )