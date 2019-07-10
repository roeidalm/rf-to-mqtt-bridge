#include <Arduino.h> //for "String" class

class WifiData
{
public:
    int getCounterIniz();
    String getSSIDName();
    WifiData(String wifiSSID,
             String wifiPassword,
             String mqttUserName,
             String mqttPassword,
             String mqttServer,
             int Port);

private:
    static int _counter;
    String _wifiPassword;
    String _mqttUserName;
    String _mqttPassword;
    String _mqttServer;
    String _wifiSSID;
    int _port;
};

//parameterized constructor
WifiData::WifiData(String wifiSSID,
                   String wifiPassword,
                   String mqttUserName,
                   String mqttPassword,
                   String mqttServer,
                   int Port = 1883)
{
    _wifiSSID = wifiSSID;
    _wifiPassword = wifiPassword;
    _mqttUserName = mqttUserName;
    _mqttPassword = mqttPassword;
    _mqttServer = mqttServer;
    _port = Port;
    _counter++;
}
int WifiData::getCounterIniz()
{
    return 0;
}
String WifiData::getSSIDName()
{
    return "s";
}
