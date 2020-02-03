add_library( slam_project STATIC
        camera.cpp
        frame.cpp
        mappoint.cpp
        map.cpp
        config.cpp
	g2o_types.cpp
        visual_odometry.cpp
)

target_link_libraries( slam_project
        ${THIRD_PARTY_LIBS}
        )
