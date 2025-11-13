px4_add_module(
	MODULE drivers__distance_sensor__ydlidar_g2
	MAIN ydlidar_g2
	COMPILE_FLAGS
	SRCS
		ydlidar_g2.cpp
		ydlidar_g2.hpp
		ydlidar_g2_main.cpp
	DEPENDS
		drivers_rangefinder
		px4_work_queue
	MODULE_CONFIG
		module.yaml
	)
