idf_component_register(
    SRCS "microros_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component bsp vision
)
