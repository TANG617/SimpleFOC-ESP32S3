/**
 * Comprehensive BLDC motor control example using magnetic sensor
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * See more info in docs.simplefoc.com/commander_interface
 */
//AVP 3 AVF0.5
#include <SimpleFOC.h>
#define Serial Serial0

// magnetic sensor instance - SPI
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4,1);
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, _NC, 45, 46);


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
// void onPID(char*cmd){commander.pid(&pid, cmd)}
// commander.pid(&pid, cmd)

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
// /**
//  *
//  * Velocity motion control example
//  * Steps:
//  * 1) Configure the motor and magnetic sensor
//  * 2) Run the code
//  * 3) Set the target velocity (in radians per second) from serial terminal
//  *
//  *
//  * By using the serial terminal set the velocity value you want to motor to obtain
//  *
//  */
// #include <SimpleFOC.h>
// #define Serial Serial0

// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// TwoWire I2Cone = TwoWire(0);

// BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4,1);

// // velocity set point variable
// float target_velocity = 1;
// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

// void setup() {
//     // use monitoring with serial
//   Serial.begin(115200);
//   Serial.println("OK");

//   // initialise magnetic sensor hardware
//   I2Cone.begin(10,9, 400000UL);   //SDA0,SCL0=10,9
//   // initialise magnetic sensor hardware
//   sensor.init(&I2Cone);
//   // sensor.init();
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 12;
//   driver.init();
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // set motion control loop to be used
//   motor.controller = MotionControlType::velocity;

//   // contoller configuration
//   // default parameters in defaults.h

//   // velocity PI controller parameters
//   motor.PID_velocity.P = 1;
//   motor.PID_velocity.I = 1.2;
//   // motor.PID_velocity.D = 0.01f;
//   // default voltage_power_supply
//   motor.voltage_limit = 12;
//   // jerk control using voltage voltage ramp
//   // default value is 300 volts per sec  ~ 0.3V per millisecond
//   motor.PID_velocity.output_ramp = 1000;

//   // velocity low pass filtering
//   // default 5ms - try different values to see what is the best.
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.05f;


//   // comment out if not needed
//   motor.useMonitoring(Serial);
//   motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 

//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   motor.initFOC();

//   // add target command T
//   command.add('T', doTarget, "target velocity");
//   motor.useMonitoring(Serial);
//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target velocity using serial terminal:"));
//   _delay(1000);
// }

// void loop() {
//   // main FOC algorithm function
//   // the faster you run this function the better
//   // Arduino UNO loop  ~1kHz
//   // Bluepill loop ~10kHz
//   motor.loopFOC();

//   // motor.monitor();

//   // Motion control function
//   // velocity, position or voltage (defined in motor.controller)
//   // this function can be run at much lower frequency than loopFOC() function
//   // You can also use motor.move() and set the motor.target in the code
//   motor.move(target_velocity);
//   Serial.print(sensor.getVelocity()/(2*PI));
//   // Serial.print(" ");
//   // Serial.print(sensor.getAngle());
//   Serial.print("\n");
//   // function intended to be used with serial plotter to monitor motor variables
//   // significantly slowing the execution down!!!!
//   // motor.monitor();

//   // user communication
//   command.run();
// }
