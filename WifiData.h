#include <string.h>

class WifiData
{
public:
    String WifiSSID;
private:
    String WifiPassword;
    String MQTTUserName;
    String MQTTPassword;
    String mqtt_server;
    int port = 1883;

public:
    //parameterized constructor
    WifiData(String wifiSSID,
             String wifiPassword,
             String mqttUserName,
             String mqttPassword,
             String mqttServer,
             int Port = 1883)
    {
        WifiSSID = wifiSSID;
        WifiPassword = wifiPassword;
        MQTTUserName = mqttUserName;
        MQTTPassword = mqttPassword;
        mqtt_server = mqttServer;
        port = Port;
    }
};