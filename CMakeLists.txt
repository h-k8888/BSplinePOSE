cmake_minimum_required(VERSION 3.25)
project(bsplines)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization thread)

##########sparse_block_matrix##########
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/sparse_block_matrix/cmake)
find_package(SuiteSparse REQUIRED)
set(SBM_INCLUDE_DIR sparse_block_matrix/include)
set(SBM_SRC_DIR sparse_block_matrix/src)
file(GLOB sbm_files
        ${SBM_SRC_DIR}/matrix_structure.cpp
        ${SBM_SRC_DIR}/sparse_helper.cpp
        ${SBM_SRC_DIR}/marginal_covariance_cholesky.cpp
        )

######################################
##########Schweizer-Messer############
######################################

######### Common #########
set(SM_COMMON_INCLUDE_DIR Schweizer-Messer/sm_common/include)
set(SM_COMMON_SRC_DIR Schweizer-Messer/sm_common/src)

######### Eigen #########
set(SM_EIGEN_INCLUDE_DIR Schweizer-Messer/sm_eigen/include)
set(SM_EIGEN_SRC_DIR Schweizer-Messer/sm_eigen/src)

######### Boost #########
set(SM_BOOST_INCLUDE_DIR Schweizer-Messer/sm_boost/include)
set(SM_BOOST_SRC_DIR Schweizer-Messer/sm_boost/src)

######### Random #########
set(SM_RANDOM_INCLUDE_DIR Schweizer-Messer/sm_random/include)
set(SM_RANDOM_SRC_DIR Schweizer-Messer/sm_random/src)
file(GLOB random_files
        ${SM_RANDOM_SRC_DIR}/random.cpp
        )

######### Kinematics #########
set(SM_KINEMATICS_INCLUDE_DIR Schweizer-Messer/sm_kinematics/include)
set(SM_KINEMATICS_SRC_DIR Schweizer-Messer/sm_kinematics/src)
file(GLOB kinematics_files
        ${SM_KINEMATICS_SRC_DIR}/RotationalKinematics.cpp
        ${SM_KINEMATICS_SRC_DIR}/RotationVector.cpp
        ${SM_KINEMATICS_SRC_DIR}/quaternion_algebra.cpp
        ${SM_KINEMATICS_SRC_DIR}/rotations.cpp
        ${SM_KINEMATICS_SRC_DIR}/transformations.cpp
        ${SM_KINEMATICS_SRC_DIR}/RotationalKinematics.cpp
        ${SM_KINEMATICS_SRC_DIR}/RotationVector.cpp
        ${SM_KINEMATICS_SRC_DIR}/EulerAnglesZYX.cpp
        ${SM_KINEMATICS_SRC_DIR}/EulerAnglesYawPitchRoll.cpp
        ${SM_KINEMATICS_SRC_DIR}/EulerRodriguez.cpp
        ${SM_KINEMATICS_SRC_DIR}/Transformation.cpp
        ${SM_KINEMATICS_SRC_DIR}/homogeneous_coordinates.cpp
        ${SM_KINEMATICS_SRC_DIR}/HomogeneousPoint.cpp
        ${SM_KINEMATICS_SRC_DIR}/UncertainTransformation.cpp
        ${SM_KINEMATICS_SRC_DIR}/UncertainHomogeneousPoint.cpp
        ${SM_KINEMATICS_SRC_DIR}/three_point_methods.cpp
        ${SM_KINEMATICS_SRC_DIR}/EulerAnglesZXY.cpp
        )

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
        ${SM_KINEMATICS_INCLUDE_DIR}
        ${SM_COMMON_INCLUDE_DIR}
        ${SM_EIGEN_INCLUDE_DIR}
        ${SM_BOOST_INCLUDE_DIR}
        ${SM_RANDOM_INCLUDE_DIR}
        ${SBM_INCLUDE_DIR}
)

file(GLOB splines_files
        src/BSpline.cpp
        src/BSplinePose.cpp
        )

add_executable(${PROJECT_NAME} main.cpp
        ${splines_files}
        ${kinematics_files}
        ${random_files}
        ${sbm_files}
        )

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        )