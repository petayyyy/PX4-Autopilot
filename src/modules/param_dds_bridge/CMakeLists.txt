px4_add_module(
	MODULE modules__param_dds_bridge
	MAIN param_dds_bridge
	SRCS
		ParamDdsBridge.cpp
		ParamDdsBridge.hpp
	DEPENDS
		px4_work_queue
	)
