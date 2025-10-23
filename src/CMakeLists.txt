# ESP-IDF组件注册
idf_component_register(
    SRCS
        "main.c"
        "lvgl_demo_ui.c"
        "common/config/config_manager.c"
        "common/wifi_manager/wifi_manager.c"
    INCLUDE_DIRS
        "."
        "../include"
        "common/config"
        "common/wifi_manager"
)