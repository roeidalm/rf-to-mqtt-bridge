
#include <WiFi.h>
#include <PubSubClient.h>
#include <RCSwitch.h>
#include <ArduinoJson.h>
#include <ELECHOUSE_CC1101_RCS_DRV.h>


const char ssid[]  = "XXX";
const char password[]  = "XXX";
const char mqtt_server[]  = "XXX";
const int port = 1883;
const char clientID[]  = "RouterEsp32_Main";
const char inTopic[] = "GateWayIn";
const char OutTopic[] = "GateWayOut";
const char complateTopic[] = "GateWayComplate";


WiFiClient espClient;
PubSubClient client(espClient);
RCSwitch myRfSwitch = RCSwitch();
int LED = 2;
int esp;
int RFTRANSMITPIN;
int RFRECIEVEPIN;


char msg[50];
uint32_t FirstFreeSpeace = esp_get_free_heap_size();
unsigned long timer_last_keep_ALIVE = 0;

void setup() {


  Serial.begin(115200);
#ifdef ESP32
  esp = 2; RFTRANSMITPIN = 2; RFRECIEVEPIN = 4; // for esp32! Transmit on GPIO pin 2.
#elif ESP8266
  esp = 1; RFTRANSMITPIN = 2; RFRECIEVEPIN = 4;  // for esp8266! Transmit on pin 5 = D1
#else
  esp = 0; RFTRANSMITPIN = 2; RFRECIEVEPIN = 4;  // for Arduino! Transmit on pin 6.
#endif
  pinMode(LED, OUTPUT);

  //CC1101 Settings:                (Settings with "//" are optional!)
  ELECHOUSE_cc1101.setESP8266(esp);  // esp8266 & Arduino SPI pin settings. Don´t change this line!
  ELECHOUSE_cc1101.setRxBW(11);//16     // set Receive filter bandwidth (default = 812khz) 1 = 58khz, 2 = 67khz, 3 = 81khz, 4 = 101khz, 5 = 116khz, 6 = 135khz, 7 = 162khz, 8 = 203khz, 9 = 232khz, 10 = 270khz, 11 = 325khz, 12 = 406khz, 13 = 464khz, 14 = 541khz, 15 = 650khz, 16 = 812khz.
  ELECHOUSE_cc1101.setMHZ(433.92); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.Init(PA10);    // must be set to initialize the cc1101! set TxPower  PA10, PA7, PA5, PA0, PA_10, PA_15, PA_20, PA_30.



  myRfSwitch.enableTransmit(RFTRANSMITPIN);
  myRfSwitch.enableReceive(RFRECIEVEPIN);
  ELECHOUSE_cc1101.SetRx();  // set Recive on

  // Optional set protocol (default is 1, will work for most outlets)
  // mySwitch.setProtocol(2);

  // Optional set pulse length.
  // mySwitch.setPulseLength(320);

  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);


  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);


  timer_last_keep_ALIVE = millis() - 1800001;

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  Timers();
  if (myRfSwitch.available()) {
    unsigned long rfButton = myRfSwitch.getReceivedValue();

    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& JSONencoder = JSONbuffer.createObject();
    JSONencoder["device"] = clientID;
    JSONencoder["freeSpeace"] = esp_get_free_heap_size();    
    JsonArray& values = JSONencoder.createNestedArray("values");

    values.add(rfButton);

    char JSONmessageBuffer[100];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending message to MQTT topic..");
    Serial.println(JSONmessageBuffer);

    client.publish(OutTopic, JSONmessageBuffer);
    Serial.println("message Sent  to MQTT:");
    Serial.println("Topic: " + String(OutTopic));
    Serial.println("command: " + String(rfButton));
    timer_last_keep_ALIVE = millis();
    delay(750);
  }
  myRfSwitch.resetAvailable();
}

void Timers() {

  unsigned long now = millis();
  uint32_t freeSpeace = esp_get_free_heap_size();

  if (now - timer_last_keep_ALIVE > 1800000) {
    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& JSONencoder = JSONbuffer.createObject();
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

  /*
     if ( (freeSpeace-FirstFreeSpeace) > ((FirstFreeSpeace * 110) / 100)) {
     StaticJsonBuffer<300> JSONbuffer;
     JsonObject& JSONencoder = JSONbuffer.createObject();
     JSONencoder["device"] = clientID;
     JSONencoder["firstSpeaceUse"] = FirstFreeSpeace;
     JSONencoder["moreThen10PercentFromTheStart"] = freeSpeace;
     char JSONmessageBuffer[100];
     JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
     Serial.println("Sending Timer to MQTT topic..");
     Serial.println(JSONmessageBuffer);
     client.publish(OutTopic, JSONmessageBuffer);
    }*/
}
void LEDblink() {
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
  delay(150);
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
}
void LEDblinkShort() {
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
}
void setup_wifi() {
  LEDblink();

  delay(1000);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println("Setting WiFi ");
  WiFi.mode(WIFI_STA);
  Serial.println("Disconnetting WiFi ");
  WiFi.disconnect();
  delay(100);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)  {
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
  LEDblinkShort();
}




void callback(char* topic, byte* payload, unsigned int length) {
  // Conver the incoming byte array to a string
  payload[length] = '\0'; // Null terminator used to terminate the char array
  char* message = (char*)payload;
  //long rfmsgFromMQTT = atol(message);


  long rfmsgfromMQTT = atol(message);


  // cc1101 set Transmit on
  ELECHOUSE_cc1101.SetTx();
  myRfSwitch.send(rfmsgfromMQTT, 24);
  delay(500);
  ELECHOUSE_cc1101.SetRx();  // set Recive on

  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["device"] = clientID;
  JSONencoder["freeSpeace"] = esp_get_free_heap_size();
  JsonArray& values = JSONencoder.createNestedArray("values");

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




void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    LEDblinkShort();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID)) {
      Serial.println("connected");
      client.subscribe(inTopic);
      Serial.println("Topic: " + String(inTopic));
    } else {
      client.setServer(mqtt_server, port);
      if (client.connect(clientID)) {
        Serial.println("connected");
        client.subscribe(inTopic);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
    }
    delay(5000);
  }
  LEDblink();
}
