
//AVP 3 AVF0.5
#include <SimpleFOC.h>
#define Serial Serial0

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4,1);
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, _NC, 45, 46);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }


void setup() {

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
  motor.controller = MotionControlType::velocity;

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
  // monitoring port
  Serial.begin(115200);
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
  motor.target = 2;

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
  motor.move();

  // user communication
  command.run();
}
