add_library(base ${LEGO_LIB_TYPE}
        base_vertex.h
        base_vertex.cpp
        base_edge.h
        base_edge.cpp
        cost_function.h
        cost_function.cpp
        problem.h
        problem.cpp
        utility.h
        utility.cpp
        timer.h
        )

set_target_properties(base PROPERTIES OUTPUT_NAME ${LIB_PREFIX}base)
target_link_libraries(base PUBLIC ${LEGO_EIGEN3_EIGEN_TARGET})

install(TARGETS base
        EXPORT ${LEGO_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
        INCLUDES DESTINATION ${INCLUDES_DESTINATION}
        )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/base)
