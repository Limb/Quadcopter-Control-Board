// Quadcopter Control Board
// Kenneth Lorthioir
// Frank Zulferino

#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "RTPressure.h"
#include "CalLib.h"
#include <EEPROM.h>
#include <PID_v1.h>

//#define DEBUG

#define NUM_CHANNELS 4
#define CH1 A0
#define CH2 A1
#define CH3 A2
#define CH4 A3

#define ESC_MIN 1060
#define ESC_MAX 1860

/* 
// Radio Values - Soldered Prototype
#define AILERON_MIN 1080
#define AILERON_MAX 1980
#define ELEVATOR_MIN 1076
#define ELEVATOR_MAX 1896
#define THROTTLE_MIN 1010
#define THROTTLE_MAX 1780
#define RUDDER_MIN 1080
#define RUDDER_MAX 1980
*/

// Radio Values - Circuit Board
#define AILERON_MIN 1015
#define AILERON_MAX 1790
#define ELEVATOR_MIN 1000
#define ELEVATOR_MAX 1770
#define THROTTLE_MIN 1015
#define THROTTLE_MAX 1780
#define RUDDER_MIN 980
#define RUDDER_MAX 1765

#define ARM_THROTTLE_THRESHOLD ESC_MIN + 100
#define ARM_RUDDER_THRESHOLD ESC_MIN + 100

#define DISARM_THROTTLE_THRESHOLD ARM_THROTTLE_THRESHOLD
#define DISARM_RUDDER_THRESHOLD ESC_MAX - 100

#define ESC1_CH 3
#define ESC2_CH 10
#define ESC3_CH 9
#define ESC4_CH 6

#define ROLL_P_VAL 1
#define ROLL_I_VAL 0
#define ROLL_D_VAL 0

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 0

#define YAW_P_VAL 0.5
#define YAW_I_VAL 0
#define YAW_D_VAL 0


// Create an array to hold all the readings from our channels
volatile uint16_t reading[NUM_CHANNELS];

// These values will hold the starting (Rise time) of our individual
// receiver channel pulses
volatile uint16_t ch1Start = micros(), 
                  ch2Start = micros(),
                  ch3Start = micros(), 
                  ch4Start = micros();

// Objects related to MPU-9150 and BMP180
RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object                   

// Values for level readings
double levelX = 0;
double levelY = 0;
double levelZ = 0;

// PID objects and variables
// PID target value
double pidXTarget;
double pidYTarget;
double pidZTarget;

// PID input variables
double Xaxis;
double Yaxis;
double Zaxis;

// PID output variables
double pidXOut;
double pidYOut;
double pidZOut;

// PID objects
PID xPID(&Xaxis, &pidXOut, &pidXTarget, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, DIRECT);
PID yPID(&Yaxis, &pidYOut, &pidYTarget, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, DIRECT);
PID zPID(&Zaxis, &pidZOut, &pidZTarget, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

// Servo objects
Servo ESC1,
      ESC2,
      ESC3,
      ESC4;

// Bool which holds our arm state
bool armed = false;

// Rising Interrupts
void ch1RisingInterrupt() {
    ch1Start = micros();
    attachPinChangeInterrupt(CH1, ch1FallingInterrupt, FALLING);
}

void ch2RisingInterrupt() {
    ch2Start = micros();
    attachPinChangeInterrupt(CH2, ch2FallingInterrupt, FALLING);
}

void ch3RisingInterrupt() {
    ch3Start = micros();
    attachPinChangeInterrupt(CH3, ch3FallingInterrupt, FALLING);
}

void ch4RisingInterrupt() {
    ch4Start = micros();
    attachPinChangeInterrupt(CH4, ch4FallingInterrupt, FALLING);
}

// Falling interrupts
void ch1FallingInterrupt() {
    reading[0] = micros() - ch1Start;
    attachPinChangeInterrupt(CH1, ch1RisingInterrupt, RISING);
}

void ch2FallingInterrupt() {
    reading[1] = micros() - ch2Start;
    attachPinChangeInterrupt(CH2, ch2RisingInterrupt, RISING);
}

void ch3FallingInterrupt() {
    reading[2] = micros() - ch3Start;
    attachPinChangeInterrupt(CH3, ch3RisingInterrupt, RISING);
}

void ch4FallingInterrupt() {
    reading[3] = micros() - ch4Start;
    attachPinChangeInterrupt(CH4, ch4RisingInterrupt, RISING);
}                  

void setup() {
	// Set all RC channels as inputs and enable pullups
	pinMode(CH1, INPUT_PULLUP);
  	pinMode(CH2, INPUT_PULLUP);
  	pinMode(CH3, INPUT_PULLUP);
  	pinMode(CH4, INPUT_PULLUP);

  	#ifdef DEBUG
  	// If debugging enable the serial interface
  	Serial.begin(115200);
  	Serial.println("---------------------------------------");
  	#endif

  	// Enable the i2c Wire Library
  	Wire.begin();

  	// Create the MPU9150 IMU object
	imu = RTIMU::createIMU(&settings);
	// Create the BMP180 pressure object
	pressure = RTPressure::createPressure(&settings);
	
	// Initialize the IMU and Pressure sensors 
	imu->IMUInit();
	pressure->pressureInit();
	
	// Slerp power controls the fusion of gyros, accelerometers and compass
	// Can be between 0 and 1  
	fusion.setSlerpPower(0.02);

	// Enable all the sensors		
	fusion.setGyroEnable(true);
	fusion.setAccelEnable(true);
	fusion.setCompassEnable(true);

	// Delay for 50 milliseconds to let the MPU9150 start up
	delay(50);
  
  	// Do a initial reading to get all sensors up and running
	while (imu->IMURead()) {
    	fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
 	}

 	// Get a reading of our axis and store them so we know what level is
 	levelX = fusion.getFusionPose().x() * 180/M_PI;
 	levelY = fusion.getFusionPose().y() * 180/M_PI;
 	levelZ = fusion.getFusionPose().z() * 180/M_PI;


 	// Enable the PID Systems
 	xPID.SetMode(AUTOMATIC);
 	xPID.SetOutputLimits(-30,30);
 	
 	yPID.SetMode(AUTOMATIC);
 	yPID.SetOutputLimits(-30,30);

 	zPID.SetMode(AUTOMATIC);
 	zPID.SetOutputLimits(-30,30);
  
  	// Attach the rising interrupt to all RC channels
	attachPinChangeInterrupt(CH1, ch1RisingInterrupt, RISING);
  	attachPinChangeInterrupt(CH2, ch2RisingInterrupt, RISING);
  	attachPinChangeInterrupt(CH3, ch3RisingInterrupt, RISING);
	attachPinChangeInterrupt(CH4, ch4RisingInterrupt, RISING);
}

void loop() {
	#ifdef DEBUG
  	delay(100);
  	#endif

  	while (imu->IMURead()) {                                // get the latest data if ready yet
    	fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  	}

  	#ifdef DEBUG
  	Serial.print("CH1: ");
  	Serial.print(reading[0]);
	Serial.print(" CH2: ");
	Serial.print(reading[1]);
	Serial.print(" CH3: ");
	Serial.print(reading[2]);
	Serial.print(" CH4: ");
	Serial.println(reading[3]);
	RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
	Serial.println();
	#endif

  	// Get our axis readings and convertt them to degrees (Multiply by 180/M_PI)
  	Xaxis = fusion.getFusionPose().x() * 180/M_PI;
  	Yaxis = fusion.getFusionPose().y() * 180/M_PI;
  	Zaxis = fusion.getFusionPose().z() * 180/M_PI;

  	#ifdef DEBUG
  	// If debugging, print out the PID data
	Serial.print("levelx: ");
	Serial.print(levelX);
	Serial.print(" x: ");
	Serial.print(Xaxis);
	Serial.print(" pidXOut: ");
	Serial.println(pidXOut);

	Serial.print("levely: ");
	Serial.print(levelY);
	Serial.print(" y: ");
	Serial.print(Yaxis);
	Serial.print(" pidYOut: ");
	Serial.println(pidYOut);

	Serial.print("levelz: ");
	Serial.print(levelZ);
	Serial.print(" z: ");
	Serial.print(Zaxis);
	Serial.print(" pidZOut: ");
	Serial.println(pidZOut);
	#endif

	// Map radio values into ESC usable values
	int aileron = map(reading[0], AILERON_MIN, AILERON_MAX, ESC_MIN, ESC_MAX);
	int elevator = map(reading[1], ELEVATOR_MIN, ELEVATOR_MAX, ESC_MIN, ESC_MAX);
	int throttle = map(reading[2], THROTTLE_MIN, THROTTLE_MAX, ESC_MIN, ESC_MAX);
	int rudder = map(reading[3], RUDDER_MIN, RUDDER_MAX, ESC_MIN, ESC_MAX);

	//pidXTarget = levelX + map(aileron, ESC_MIN, ESC_MAX, -15, 15);
	//pidYTarget = levelY + map(elevator, ESC_MIN, ESC_MAX, -15, 15);
	//pidZTarget = levelZ + map(rudder, ESC_MIN, ESC_MAX, -15, 15);

	pidXTarget = levelX;
	pidYTarget = levelY;
	pidZTarget = levelZ;

  	// Do the PID calculations for each axis
  	xPID.Compute();
  	yPID.Compute();
  	zPID.Compute();

  	if(armed)
  	{
  		// Check to see if we should disarm

  		// Check throttle threshold
  		if(throttle < DISARM_THROTTLE_THRESHOLD)
  		{
  			// Check rudder threshhold
  			if(rudder > DISARM_RUDDER_THRESHOLD)
  			{
  				// Disarm the device

  				// Detach the ESC's
  				ESC1.detach();
    		  	ESC2.detach();
      			ESC3.detach();
      			ESC4.detach();

  				#ifdef DEBUG
  				Serial.println("DISARMED");
  				#endif

  				armed = false;
  			}
  		}

  		int va = throttle-pidXOut+pidYOut+pidZOut;
  		int vb = throttle+pidXOut+pidYOut-pidZOut;
  		int vc = throttle+pidXOut-pidYOut-pidZOut;
  		int vd = throttle-pidXOut-pidYOut+pidZOut;	

		va = constrain(va, 1060, 1860);
	    vb = constrain(vb, 1060, 1860);
	    vc = constrain(vc, 1060, 1860);
	    vd = constrain(vd, 1060, 1860);


	    ESC1.writeMicroseconds(va);
	    ESC2.writeMicroseconds(vb);
	    ESC3.writeMicroseconds(vc);
	    ESC4.writeMicroseconds(vd);
	}

  	else 
  	{	
  		// Arm the device if the thresholds are met

  		// Check throttle threshold
  		if(reading[2] < ARM_THROTTLE_THRESHOLD && reading[2] > THROTTLE_MIN)
  		{
  			if(reading[3] < ARM_RUDDER_THRESHOLD && reading[3] > RUDDER_MIN)
  			{
  				// Arm the device

  				// Attach the ESC's
  				ESC1.attach(ESC1_CH);
      			ESC2.attach(ESC2_CH);
			    ESC3.attach(ESC3_CH);
			    ESC4.attach(ESC4_CH);

  				#ifdef DEBUG
  				Serial.println("ARMED");
  				#endif

  				armed = true;
  			}
  		}
  	}
}
