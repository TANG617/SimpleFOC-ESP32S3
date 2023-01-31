
//AVP 3 AVF0.5
#include <SimpleFOC.h>
#define Serial Serial0
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

DeserializationError json_error;
DynamicJsonDocument doc_json(1024);
char input_json[1000];
//WiFi
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
int target_angle = 0;

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
  Serial.print("angle:");
  target_angle = doc_json["angle"];
  Serial.println(target_angle);
  
}

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4,1);
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, _NC, 45, 46);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd)
{ 
  command.motor(&motor, cmd); 
}


void setup() {
    // monitoring port
  Serial.begin(115200);
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
     String client_id = "esp32-client-";
     client_id += String(WiFi.macAddress());
     Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
     if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
         Serial.println("Public emqx mqtt broker connected");
     } else {
         Serial.print("failed with state ");
         Serial.print(client.state());
         delay(2000);
     }
 }
 // publish and subscribe
 client.publish(topic, "Motor Online!\n");
 client.subscribe(topic);

  // initialise magnetic sensor hardware
  // sensor.init();
  I2Cone.begin(10,9, 400000UL);   //SDA0,SCL0=10,9
  // initialise magnetic sensor hardware
  sensor.init(&I2Cone);
  // link the motor to the sensor
  motor.linkSensor(&sensor);


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;

  // set control loop type to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration based on the control type
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 50;

  // use monitoring with serial for motor init

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC

  // 初始化电流检测
  current_sense.init();
  // 连接电流检测和电机
  motor.linkCurrentSense(&current_sense);
  current_sense.skip_align = true;
  motor.initFOC();

  // set the inital target value
  motor.target = target_angle;

  // define the motor id
  command.add('A', onMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  // motor.move();
  client.loop();
  motor.move(target_angle);
  // user communication
  command.run();
  // Serial.println(target_angle);
}
