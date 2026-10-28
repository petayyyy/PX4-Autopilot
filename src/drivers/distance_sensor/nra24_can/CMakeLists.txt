# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
    MODULE drivers__distance_sensor__nra24_can
    MAIN nra24_can
    STACK_MAIN 2500
    SRCS NRA24Can.cpp
    INCLUDES ${PX4_SOURCE_DIR}/src/drivers/uavcan
    DEPENDS drivers_rangefinder
)
# UAVCAN is declared later in the board's module traversal. CMake resolves
# this target at generation time, after both modules have been declared.
target_link_libraries(drivers__distance_sensor__nra24_can PRIVATE drivers__uavcan)
