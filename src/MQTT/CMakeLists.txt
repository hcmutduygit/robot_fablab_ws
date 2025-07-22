cmake_minimum_required(VERSION 3.0.2)

# Create library for MQTT publisher
add_library(mqtt_publisher
    name_publisher.cpp
)

# Set include directories
target_include_directories(mqtt_publisher PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Install headers
install(FILES name_publisher.h
    DESTINATION include/MQTT
)
