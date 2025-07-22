#include <iostream>
#include <string>
#include <cstdlib>

class MQTTPublisher {
private:
    std::string mqtt_host;
    int mqtt_port;
    std::string mqtt_topic;

public:
    // Constructor
    MQTTPublisher(const std::string& host = "192.168.0.130", 
                  int port = 1883, 
                  const std::string& topic = "robot/attendance") 
        : mqtt_host(host), mqtt_port(port), mqtt_topic(topic) {}

    // Function to publish MQTT message
    bool publishMessage(const std::string& user_name, const std::string& mqtt_msg) {
        std::cout << "Publishing MQTT message for " << user_name << ": " << mqtt_msg << std::endl;
        
        // Create Python command to publish MQTT message
        std::string command = "python3 -c \"";
        command += "import paho.mqtt.client as mqtt; ";
        command += "MQTT_HOST='" + mqtt_host + "'; ";
        command += "MQTT_PORT=" + std::to_string(mqtt_port) + "; ";
        command += "MQTT_TOPIC='" + mqtt_topic + "'; ";
        command += "MQTT_MSG='" + mqtt_msg + "'; ";
        command += "mqttc=mqtt.Client(); ";
        command += "mqttc.connect(MQTT_HOST, MQTT_PORT, 5); ";
        command += "mqttc.publish(MQTT_TOPIC, MQTT_MSG); ";
        command += "mqttc.disconnect(); ";
        command += "print('Published: ' + MQTT_MSG)\"";
        
        // Execute command
        int result = system(command.c_str());
        
        if (result == 0) {
            std::cout << "MQTT message sent successfully!" << std::endl;
            return true;
        } else {
            std::cout << "Failed to send MQTT message!" << std::endl;
            return false;
        }
    }

    // Static function for easy access
    static bool publishUserMessage(const std::string& user_name, const std::string& mqtt_msg) {
        MQTTPublisher publisher;
        return publisher.publishMessage(user_name, mqtt_msg);
    }
};

// C-style function wrapper for easy integration
extern "C" {
    bool publish_mqtt_message(const char* user_name, const char* mqtt_msg) {
        return MQTTPublisher::publishUserMessage(std::string(user_name), std::string(mqtt_msg));
    }
}
