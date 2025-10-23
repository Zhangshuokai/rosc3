# ESP-IDF组件注册
idf_component_register(
    SRCS
        "main.c"
        "lvgl_demo_ui.c"
        "common/config/config_manager.c"
        "common/wifi_manager/wifi_manager.c"
        # 暂时禁用ROS通信模块（需要micro-ros库配置）
        # "common/ros_comm/ros_comm.c"
        # "common/ros_comm/micro_ros_transport.c"
        "common/oled/oled_display.c"
        "common/oled/oled_ui.c"
    INCLUDE_DIRS
        "."
        "../include"
        "common/config"
        "common/wifi_manager"
        # "common/ros_comm"
        "common/oled"
    REQUIRES
        driver
        esp_lcd
        lvgl
)