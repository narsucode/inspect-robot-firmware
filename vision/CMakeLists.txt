idf_component_register(
    SRCS "vision_cam.c" "vision_gimbal.c" "vision_task.c"
    INCLUDE_DIRS "."
    REQUIRES bsp driver
)
