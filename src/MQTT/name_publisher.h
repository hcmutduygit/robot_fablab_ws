#ifndef MQTT_NAME_PUBLISHER_H
#define MQTT_NAME_PUBLISHER_H

#include <string>

class MQTTPublisher {
private:
    std::string mqtt_host;
    int mqtt_port;
    std::string mqtt_topic;

public:
    // Constructor
    MQTTPublisher(const std::string& host = "192.168.0.130", 
                  int port = 1883, 
                  const std::string& topic = "robot/attendance");

    // Function to publish MQTT message
    bool publishMessage(const std::string& user_name, const std::string& mqtt_msg);

    // Static function for easy access
    static bool publishUserMessage(const std::string& user_name, const std::string& mqtt_msg);
};

// C-style function wrapper for easy integration
extern "C" {
    bool publish_mqtt_message(const char* user_name, const char* mqtt_msg);
}

#endif // MQTT_NAME_PUBLISHER_H
