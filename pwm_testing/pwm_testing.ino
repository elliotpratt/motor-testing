#include <SimpleFOC.h>

// BLDCMotor(int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance (ohms)
// - KV            - motor kv rating (rpm/V)
// - L             - motor phase inductance (H)
BLDCMotor motor = BLDCMotor(4, 5.14, 5120, 0.000127);

// BLDCDriver3PWM(pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6);

// NOTE: built-in encoder functionality assumes quadrature encoder
// Encoder(pin_A, pin_B, PPR)
// FIXME, figure out PPR
Encoder encoder = Encoder(2, 4, 2048);
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}

// gain options of 0.25, 0.5, 1, or 2 V/A --> 250, 500, 1000, or 2000 mV/A
// FIXME, need to create adjustment function in library
//  LowsideCurrentSense(mVpA, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(2000, A0, A1, A2);

// angle set point variable
float target_angle = 0;

// instantiate sensor
void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // driver init
  driver.init();
  // link the motor to the driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::angle;

  //******************************

  // link the driver with the current sense
  current_sense.linkDriver(&driver);
  // link the motor to current sense
  motor.linkCurrentSense(&current_sese);

  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  //default voltage_power_supply
  motor.voltage_limit = 6;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  // default P=20
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 4;
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // init current sense
  current_sense.init();

  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial);
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // Either voltage, angle (in radians) or velocity based on the motor.controller
  motor.move();

  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();
}
