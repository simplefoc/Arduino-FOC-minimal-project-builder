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
 * To check the config value just enter the command letter.
 * For example: - to read velocity PI controller P gain run: P
 *              - to set velocity PI controller P gain  to 1.2 run: P1.2
 * 
 * To change the target value just enter a number in the terminal:
 * For example: - to change the target value to -0.1453 enter: -0.1453
 *              - to get the current target value enter: V3 
 * 
 * List of commands:
 *  - P: velocity PID controller P gain
 *  - I: velocity PID controller I gain
 *  - D: velocity PID controller D gain
 *  - R: velocity PID controller voltage ramp
 *  - F: velocity Low pass filter time constant
 *  - K: angle P controller P gain
 *  - N: angle P controller velocity limit
 *  - L: system voltage limit
 *  - C: control loop 
 *    - 0: voltage 
 *    - 1: velocity 
 *    - 2: angle
 *  - V: get motor variables
 *    - 0: currently set voltage
 *    - 1: current velocity
 *    - 2: current angle
 *    - 3: current target value
 * 
 */
#include "src/BLDCMotor.h"
#include "src/drivers/BLDCDriver3PWM.h"
#include "src/sensors/MagneticSensorSPI.h"

// SPI magnetic sensor instance
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(12, 13, 14, 27);

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set control loop type to be used
  motor.controller = ControlType::voltage;

  // contoller configuration based on the control type 
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

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
  motor.initFOC();

  // set the inital target value
  motor.target = 2;


  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor commands sketch | Initial motion control > torque/voltage : target 2V.");
  
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
  motor.command(serialReceiveUserCommand());
}

// utility function enabling serial communication the user
String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}