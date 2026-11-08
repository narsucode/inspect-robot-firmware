idf_component_register(
    SRCS "app_test.c"
    INCLUDE_DIRS "."
    REQUIRES bsp driver esp_timer
)
