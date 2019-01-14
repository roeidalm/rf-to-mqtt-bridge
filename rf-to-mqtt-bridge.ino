
#include <WiFi.h>
#include <PubSubClient.h>
#include <RCSwitch.h>
#include <ArduinoJson.h>
#include <ELECHOUSE_CC1101_RCS_DRV.h>
#include "config.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define MQTT_MAX_PACKET_SIZE 500 //dons't check it yet, but I think it will let you to send longer data to MQTT
/*I take Main from config
const char ssid[]  = "XXX";
const char password[]  = "XXX";
const char mqtt_server[]  = "XXXX";
const int port = 1883;
const char clientID[]  = "RF2Bridge";
const char inTopic[] = "GateWayIn";
const char OutTopic[] = "GateWayOut";
const char complateOutTopic[] = "complateGateWayOut";
const char complateTopic[] = "GateWayComplate";
*/

WiFiClient espClient;
PubSubClient client(espClient);
RCSwitch myRfSwitch = RCSwitch();
//TaskHandle_t Task1;
int LED = 13;
int esp;

long arrayOfRfs[150];

char msg[50];
uint32_t FirstFreeSpeace = esp_get_free_heap_size();
unsigned long timer_last_keep_ALIVE = 0;
void setup()
{
  int RFTRANSMITPIN;
  int RFRECIEVEPIN;
  Serial.begin(115200);
#ifdef ESP32
  esp = 2;
  RFTRANSMITPIN = 2;
  RFRECIEVEPIN = 4; // for esp32! Transmit on GPIO pin 2.
#elif ESP8266
  esp = 1;
  RFTRANSMITPIN = 2;
  RFRECIEVEPIN = 4; // for esp8266! Transmit on pin 5 = D1
#else
  esp = 0;
  RFTRANSMITPIN = 2;
  RFRECIEVEPIN = 4; // for Arduino! Transmit on pin 6.
#endif
  pinMode(LED, OUTPUT);

  //CC1101 Settings:                (Settings with "//" are optional!)
  ELECHOUSE_cc1101.setESP8266(esp); // esp8266 & Arduino SPI pin settings. Don´t change this line!
  //ELECHOUSE_cc1101.setRxBW(16);     // set Receive filter bandwidth (default = 812khz) 1 = 58khz, 2 = 67khz, 3 = 81khz, 4 = 101khz, 5 = 116khz, 6 = 135khz, 7 = 162khz, 8 = 203khz, 9 = 232khz, 10 = 270khz, 11 = 325khz, 12 = 406khz, 13 = 464khz, 14 = 541khz, 15 = 650khz, 16 = 812khz.
  ELECHOUSE_cc1101.setMHZ(433.92); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.Init(PA10);     // must be set to initialize the cc1101! set TxPower  PA10, PA7, PA5, PA0, PA_10, PA_15, PA_20, PA_30.

  //set the mode to rx
  myRfSwitch.enableTransmit(RFTRANSMITPIN);
  myRfSwitch.enableReceive(RFRECIEVEPIN);
  ELECHOUSE_cc1101.SetRx(); // set Recive on

  //starting the wifi and the MQTT
  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);

  timer_last_keep_ALIVE = millis() - 1800001;

  //OTA
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.setHostname(clientID);
  ArduinoOTA.begin();
}

void loop()
{

  ArduinoOTA.handle();

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  Timers();
  //waiting to get signal
  if (myRfSwitch.available())
  {
    unsigned long rfButton = myRfSwitch.getReceivedValue();
    int i;
    int indexOfZero = 0;
    bool rfNumExist = false;
    //cheking if the signal exist
    for (i = 0; i < 150; i = i + 1)
    {
      if (rfButton == arrayOfRfs[i])
      {
        rfNumExist = true;
        break;
      }
      else if (arrayOfRfs[i] == 0 && indexOfZero == 0)
      {
        indexOfZero = i;
      }
    }
    //if not exist adding to the array and send it
    if (!rfNumExist)
    {
      arrayOfRfs[indexOfZero] = rfButton;
      Serial.println("adding number");
      Serial.println(rfButton);

      DynamicJsonBuffer JSONbuffer;

      JsonObject &JSONencoder = JSONbuffer.createObject();
      JSONencoder["device"] = clientID;
      JSONencoder["freeSpeace"] = esp_get_free_heap_size();
      JsonArray &values = JSONencoder.createNestedArray("values");
      for (i = 0; i < 150; i = i + 1)
      {
        if (arrayOfRfs[i] != 0)
        {
          values.add(arrayOfRfs[i]);
        }
      }
      char JSONmessageBuffer[100];
      JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
      Serial.println("Sending message to MQTT topic..");
      Serial.println(JSONmessageBuffer);

      client.publish(OutTopic, JSONmessageBuffer);
      Serial.println("message Sent  to MQTT:");
      Serial.println("Topic: " + String(OutTopic));
      Serial.println("command: " + String(rfButton));
      timer_last_keep_ALIVE = millis();
    }
    else
    {
      Serial.println("the number is exist");
    }
  }
  myRfSwitch.resetAvailable();
}
void Timers()
{

  unsigned long now = millis();
  uint32_t freeSpeace = esp_get_free_heap_size();

  if (now - timer_last_keep_ALIVE > 1800000)
  {
    StaticJsonBuffer<300> JSONbuffer;
    JsonObject &JSONencoder = JSONbuffer.createObject();
    JSONencoder["device"] = clientID;
    JSONencoder["freeSpeace"] = esp_get_free_heap_size();
    JSONencoder["IM-ALIVE"] = true;
    char JSONmessageBuffer[100];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending Timer to MQTT topic..");
    Serial.println(JSONmessageBuffer);
    client.publish(OutTopic, JSONmessageBuffer);
    timer_last_keep_ALIVE = now;
  }
}
void LEDblink()
{
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
  delay(150);
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
}
void LEDblinkShort()
{
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
}
void setup_wifi()
{
  LEDblink();

  delay(1000);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println("Setting WiFi ");
  WiFi.mode(WIFI_STA);
  Serial.println("Disconnetting WiFi ");
  WiFi.disconnect();
  delay(100);
  WiFi.setHostname("RF2Bridge");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    Serial.println(WiFi.localIP());
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    LEDblinkShort();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED, HIGH);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  // Conver the incoming byte array to a string
  payload[length] = '\0'; // Null terminator used to terminate the char array
  char *message = (char *)payload;
  long rfmsgfromMQTT = atol(message);
  //check if the topic is about done the task in the MQTT
  if (strcmp(topic, complateOutTopic) == 0)
  {
    int i;
    for (i = 0; i < 150; i = i + 1)
    {
      if (rfmsgfromMQTT == arrayOfRfs[i])
      {
        arrayOfRfs[i] = 0;
        break;
      }
    }
  }
  //check if the topic is about sending rf signal
  if (strcmp(topic, inTopic) == 0)
  {
    // cc1101 set Transmit on
    ELECHOUSE_cc1101.SetTx();
    myRfSwitch.send(rfmsgfromMQTT, 24);
    ELECHOUSE_cc1101.SetRx(); // set Recive on

    StaticJsonBuffer<300> JSONbuffer;
    JsonObject &JSONencoder = JSONbuffer.createObject();
    JSONencoder["device"] = clientID;
    JSONencoder["freeSpeace"] = esp_get_free_heap_size();
    JsonArray &values = JSONencoder.createNestedArray("values");

    values.add(rfmsgfromMQTT);

    char JSONmessageBuffer[100];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending message to MQTT topic..");
    Serial.println(JSONmessageBuffer);

    client.publish(complateTopic, JSONmessageBuffer);
    timer_last_keep_ALIVE = millis();

    Serial.println("Message Sent on topic: ");
    Serial.println(topic);
    Serial.println(message);
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    LEDblinkShort();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID))
    {
      Serial.println("connected");
      client.subscribe(inTopic);
      client.subscribe(complateOutTopic);
      Serial.println("inTopic: " + String(inTopic));
      Serial.println("complateOutTopic: " + String(complateOutTopic));
    }
    else
    {
      client.setServer(mqtt_server, port);
      if (client.connect(clientID))
      {
        Serial.println("connected");
        client.subscribe(inTopic);
        client.subscribe(complateOutTopic);
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
    }
    delay(5000);
  }
  digitalWrite(LED, HIGH);
}
