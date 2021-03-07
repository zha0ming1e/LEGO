add_library(slam ${LEGO_LIB_TYPE}
        edge_imu.h
        edge_imu.cpp
        edge_prior.h
        edge_prior.cpp
        edge_reprojection.h
        edge_reprojection.cpp
        imu_integration.h
        imu_integration.cpp
        vertex_inverse_depth.h
        vertex_point_xyz.h
        vertex_pose.h
        vertex_pose.cpp
        vertex_speedbias.h
        )

set_target_properties(slam PROPERTIES OUTPUT_NAME ${LIB_PREFIX}slam)
target_link_libraries(slam base)

install(TARGETS slam
        EXPORT ${LEGO_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
        INCLUDES DESTINATION ${INCLUDES_DESTINATION}
        )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/slam)
