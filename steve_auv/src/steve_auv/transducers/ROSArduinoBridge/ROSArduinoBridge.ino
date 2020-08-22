
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ISM330DHCT.h>

#include <Wire.h>
#include "MS5837.h"


/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define USE_BASE      // Enable the base controller code
#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   #define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   #define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   //#define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   //#define L298_MOTOR_DRIVER
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"



/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// liquid sensor
int ledpin=13;// initialize pin 13
int inpin=22;// initialize pin 7
int val;// define val
// int. temp sens
#define ONE_WIRE_BUS 23
// IMU pins
// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 20
#define LSM_MISO 12
#define LSM_MOSI 21
//depth sensor
MS5837 sensor;
// rangefinders
#define rangePin1 A0
#define rangePin2 A1

long distance1 = 0;
long distance2 = 0;


// power health
#define battPin0 A2    // select the input pins for the batteries
#define battPin1 A3
#define battPin2 A4
#define battPin3 A5

float battWeighting = 0.1; // define a weighting to apply to our exponential moving average calculation for battery

int abattValue0 = 0; // variable to store average value (exponential moving average) calculation
int abattValue1 = 0;
int abattValue2 = 0;
int abattValue3 = 0;

float Cell1 = 0.00; // variable to store actual cell voltages
float Cell2 = 0.00;
float Cell3 = 0.00;
float Cell4 = 0.00;

float adcVolt = 0.0048680351906158 ; // one point on the ADC equals this many volts


// motors
int RPWM=2;
int LPWM=3;
//timer 0
int L_EN=40;
int R_EN=41;

int RPWM2=4;
int LPWM2=5;
int R_EN2=38;
int L_EN2=39;

int RPWM3=6;
int LPWM3=7;
int R_EN3=36;
int L_EN3=37;

int RPWM4=8;
int LPWM4=9;
int R_EN4=34;
int L_EN4=35;

int RPWM5=10;
int LPWM5=11;
int R_EN5=32;
int L_EN5=33;

int RPWM6=12;
int LPWM6=13;
int R_EN6=30;
int L_EN6=31;

int liquidThere = 0;


// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PINGG:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

//Something from IMU
Adafruit_ISM330DHCT ism330dhcx;
/* Setup function--runs once at startup. */
void setup() {
  for(int i=0;i<42;i++){
   pinMode(i,OUTPUT);
  }
   for(int i=0;i<42;i++){
   digitalWrite(i,LOW);
  }
  delay(1000);
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  //Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ISM330DHCX test!");

  if (!ism330dhcx.begin_I2C()) {
    // if (!ism330dhcx.begin_SPI(LSM_CS)) {
    // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ISM330DHCX Found!");

  // ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCT_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  // ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2

  //rangeinder setup
  Serial.println("Starting");
  
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

 // pinMode(rangePin1, INPUT);
 // pinMode(rangePin2, INPUT);

  pinMode(ledpin,OUTPUT);// set LED pin as “output”
  pinMode(inpin,INPUT);// set button pin as “input”


  for (int i=0; i<50; i++){
    getBattVolts();
  }
}

// Initialize the motor controller if used */
void read_liquid_sensor() {
val=digitalRead(inpin);// read the level value of pin 7 and assign if to val
//Serial.println(val); // print the data from the sensor
delay(100);
if(val==LOW)// check if the button is pressed, if yes, turn on the LED
{ digitalWrite(ledpin,LOW);}
else
{ digitalWrite(ledpin,HIGH);}
}

void print_data_liquid() {
 Serial.print("liquid = ");
 Serial.print(val);
 Serial.println(" ");
}


void read_range_sensor1() {
distance1 = analogRead(rangePin1) * 5;
}

void read_range_sensor2() {
distance2 = analogRead(rangePin2) * 5;
}

void print_data_range1() {
Serial.print("distance1 = ");
Serial.print(distance1);
Serial.println(" cm");
}

void print_data_range2() {
Serial.print("distance2 = ");
Serial.print(distance2);
Serial.println(" cm");
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void int_temp() { 
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperature is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? 
    // You can have more than one IC on the same bus. 
    // 0 refers to the first IC on the wire
    delay(1000);
}

void imu() {
  
 //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);

}

void motor_move() {
       Serial.println("EN_high");
     digitalWrite(R_EN,HIGH);
     digitalWrite(L_EN,HIGH);
     digitalWrite(R_EN6,HIGH);
     digitalWrite(L_EN6,HIGH);
  delay(1000);
  for(int i=0;i<256;i++){
     analogWrite(RPWM,i);
     analogWrite(RPWM6,i);
     delay(100); 
  }
  delay(500);
  for(int i=255;i>0;i--){
    analogWrite(RPWM,i);
    analogWrite(RPWM6,i);
    // analogWrite(LPWM,255-i);
    delay(100);
  }
  

  
  delay(500);
  Serial.println("EN LOW");
  digitalWrite(R_EN,LOW);
  digitalWrite(L_EN,LOW);
  digitalWrite(R_EN6,LOW);
  digitalWrite(L_EN6,LOW);
  delay(1000);
  for(int i=0;i<256;i++){
    analogWrite(RPWM,i);
    analogWrite(RPWM6,i);
    delay(100);
  }
  delay(500);
  for(int i=255;i>0;i--){
    analogWrite(RPWM,i);
    analogWrite(RPWM6,i);
    delay(100);
  }
  delay(500);
}

void depth() {
  // Update pressure and temperature readings
  sensor.read();

  Serial.print("Pressure: "); 
  Serial.print(sensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(sensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(sensor.altitude()); 
  Serial.println(" m above mean sea level");

  delay(1000);
}

void getBattVolts() {
  // read the value from the sensor:
  abattValue0 = (analogRead(battPin0) * battWeighting) + (abattValue0 * (1-battWeighting));
  abattValue1 = (analogRead(battPin1) * battWeighting) + (abattValue1 * (1-battWeighting));
  abattValue2 = (analogRead(battPin2) * battWeighting) + (abattValue2 * (1-battWeighting));
  abattValue3 = (analogRead(battPin3) * battWeighting) + (abattValue3 * (1-battWeighting));
  // convert these values to cell voltages
  Cell1 = (adcVolt * abattValue0 * 1) ;
  Cell2 = (adcVolt * abattValue1 * 1.8298)-Cell1;
  Cell3 = (adcVolt * abattValue2 * 2.6078)-Cell2-Cell1;
  Cell4 = (adcVolt * abattValue3 * 3.7659)-Cell3-Cell2-Cell1;
}

void showBattVolts() {
  Serial.print (Cell1);
  Serial.print ("V. " );
  Serial.print (Cell2);
  Serial.print ("V. ");
  Serial.print (Cell3);
  Serial.print ("V. ");
  Serial.print (Cell4);
  Serial.print ("V. Total = " );
  Serial.println (Cell1+Cell2+Cell3+Cell4);
}




int incoming;

void loop() {
  //Call functions for random data collections
  //read_liquid_sensor();
  //print_data_liquid();
  //int_temp();
  //imu();
  //depth();
  //read_range_sensor1();
  //print_data_range1();
  //read_range_sensor2();
  //print_data_range2();
  //getBattVolts();
  //showBattVolts();
  
  delay(500);




  //while(Serial.available() > 0){
  //  incoming = Serial.read();
  //}
  //if(incoming == 1) {
   // read_liquid_sensor();
  //  print_data_liquid();
    //motor_move();
 // }

     

  while (Serial.available() > 0) {
    // Serial.print("Serial was available: ");
    // read_liquid_sensor();
     //print_data_liquid();
     //int_temp();
     Serial.print("penis");
     delay(100);
     incoming = Serial.read();
     Serial.println(incoming);
     if (incoming == 'h') {
       read_liquid_sensor();
       print_data_liquid();
    //motor_move();
     }
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
