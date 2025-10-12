add_library(${PROJECT_NAME}
        voxelslam.cpp
        BTC.cpp
        voxel_slam_nodelet.cpp
        )

# Ensure the produced library name matches nodelet_plugins.xml (libvoxel_slam.so)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "voxel_slam")

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        gtsam
        )

target_include_directories(${PROJECT_NAME} PRIVATE   
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${CERES_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_nodelet
        voxel_slam_nodelet.cpp
        )

add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME})
