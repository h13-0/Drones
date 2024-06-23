# DRIVERS_SOURCES
file(GLOB_RECURSE DRIVERS_SOURCES "./*.c")

idf_component_register(
    SRCS
        ${DRIVERS_SOURCES}
    REQUIRES
        driver
        esp_wifi
        nvs_flash
    INCLUDE_DIRS 
        "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/debugger"
        "${CMAKE_CURRENT_LIST_DIR}/imu"
        "${CMAKE_CURRENT_LIST_DIR}/motor"
        "${CMAKE_CURRENT_LIST_DIR}/nvs_flash"
        "${CMAKE_CURRENT_LIST_DIR}/wifi"
)
