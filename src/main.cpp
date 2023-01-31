#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


DeserializationError json_error;
#define Serial Serial0
DynamicJsonDocument doc_json(1024);

char input_json[1000];

// WiFi
const char *ssid = "HOME"; // Enter your WiFi name
const char *password = "T20030617";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.31.87";
const char *topic = "motor";
const char *mqtt_username = NULL;
const char *mqtt_password = NULL;
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length) {
 Serial.print("Message arrived in topic: ");
 Serial.println(topic);
 Serial.print("Message:");
 for (int i = 0; i < length; i++) {
    //  Serial.print((char) payload[i]);
    input_json[i]=((char)payload[i]);
 }
 input_json[length]='\0';
 Serial.println(input_json);
 deserializeJson(doc_json, input_json);
 Serial.print("motor1_speed:");
 int speed = doc_json["speed"];
 Serial.println(speed);
 
}

void setup() {
 // Set software serial baud to 115200;
 Serial.begin(115200);
 // connecting to a WiFi network
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.println("Connecting to WiFi..");
 }
 Serial.println("Connected to the WiFi network");
 //connecting to a mqtt broker
 client.setServer(mqtt_broker, mqtt_port);
 client.setCallback(callback);
 
 while (!client.connected()) {
     String client_id = "ESP32-";
     client_id += String(WiFi.macAddress());
     Serial.printf("The client %s connectted\n", client_id.c_str());
     if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
         Serial.println("Public emqx mqtt broker connected");
     } else {
         Serial.print("failed with state ");
         Serial.print(client.state());
         delay(2000);
     }
 }
 // publish and subscribe
 client.publish(topic, "ESP32 is Online!");
 client.subscribe(topic);
}



void loop() {
 client.loop();
}
