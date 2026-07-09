cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_touch_controller)

# The bundled esp_lvgl_port (LVGL 8 path) still calls the deprecated
# esp_lcd_touch_get_coordinates() API, which triggers a compiler
# deprecation warning we cannot fix in that managed component.  Our own
# code uses the replacement esp_lcd_touch_get_data() API; silence the
# deprecation warning for this managed component only.
idf_component_get_property(esp_lvgl_port_lib espressif__esp_lvgl_port COMPONENT_LIB)
if(esp_lvgl_port_lib)
    target_compile_options(${esp_lvgl_port_lib} PRIVATE -Wno-deprecated-declarations)
endif()
