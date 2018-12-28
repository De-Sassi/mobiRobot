/*
 Name:		mobiRobot.ino
 Created:	12/5/2018 3:21:29 PM
 Author:	delia
*/

#include <DFRobot_LCD.h>

#include <ZumoShield.h>


//#include <Wire.h>

//PINS
#define LED_PIN 13
#define LEFT_GREEN_PIN 50 
#define LEFT_YELLOW_PIN 21 
#define RIGHT_GREEN_PIN 24
#define RIGHT_YELLOW_PIN 20
//
#define COUNTS_PER_CPOMPLETE_REVOLUTION 693
#define WHEEL_CIRCUMFERENCE 119 //in mm
#define COUNT_FULL_LEFT_CIRCLE 1545 

ZumoMotors motors;

DFRobot_LCD lcd(16, 2);  //16 characters and 2 lines of show


ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
Pushbutton button(ZUMO_BUTTON);

//Encoder
int leftReadingAPhase = LOW;
int leftReadingAPhaseOld = LOW;
int rightReadingAPhase = LOW;
int rightReadingAPhaseOld = LOW;
volatile int leftEncoderValue = 0;
volatile int rightEncoderValue = 0;


void setup()
{


	

	//Start communication
	Serial.begin(9600);
	while (!Serial)
	{
		;
	}
	establishContact();
	Serial.println("Serial connection ready");

	lcd.init();
	//set LED pin
	pinMode(LED_PIN, OUTPUT);

	//set encoder pins
	//left
	pinMode(LEFT_GREEN_PIN, INPUT);
	pinMode(LEFT_YELLOW_PIN, INPUT);
	//right
	pinMode(RIGHT_GREEN_PIN, INPUT);
	pinMode(RIGHT_YELLOW_PIN, INPUT);

	//Initialize interupts
	attachInterrupt(digitalPinToInterrupt(LEFT_YELLOW_PIN), leftEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RIGHT_YELLOW_PIN), rightEncoder, CHANGE);
	// uncomment one or both of the following lines if your motors' directions need to be flipped
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

	
}


void loop()
{
	digitalWrite(LED_PIN, HIGH);
	lcd.clear();
	lcd.print("loop");
	delay(2000);


	
	lcd.clear();
	lcd.println("Turn 30");
	turnRight(30);
	delay(1000);
	lcd.clear();
	lcd.println("straight 240");
	driveDistance(240);
	delay(1000);
	lcd.clear();
	lcd.println("turn 30");
	turnLeft(30);
	delay(1000);
	lcd.clear();
	lcd.println("straight 400");
	driveDistance(400);
	digitalWrite(LED_PIN, HIGH);
	delay(10000);



	digitalWrite(LED_PIN, LOW);

	//for (int i = 0; i < 30; i++)
	//{

	//	Serial.print("Pin nr ");
	//	Serial.print(i);
	//	Serial.print("is interrrupt");
	//	Serial.println(digitalPinToInterrupt(i));
	//	delay(1000);
	//}



}

void leftEncoder()
{
	leftReadingAPhase = digitalRead(LEFT_GREEN_PIN);
	if ((leftReadingAPhase == HIGH) && (leftReadingAPhaseOld == LOW))
	{
		if (digitalRead(LEFT_YELLOW_PIN) == HIGH)
		{
			leftEncoderValue++;
		}
		else {
			leftEncoderValue--;
		}
	}
	leftReadingAPhaseOld = leftReadingAPhase;
}

void rightEncoder()
{

	rightReadingAPhase = digitalRead(RIGHT_GREEN_PIN);
	if ((rightReadingAPhase == HIGH) && (rightReadingAPhaseOld == LOW))
	{
		if (digitalRead(RIGHT_YELLOW_PIN) == HIGH)
		{
			rightEncoderValue--;
		}
		else {
			rightEncoderValue++;
		}
	}
	rightReadingAPhaseOld = rightReadingAPhase;
}

void driveDistance(int distanceInMM)
{
	int count=getCountsForDistance(distanceInMM);
	resetEncoderCounters();
	motors.setSpeeds(100, 100);
	while ((leftEncoderValue < count) || (rightEncoderValue < count))
	{
		if (rightEncoderValue >= count)
		{
			motors.setRightSpeed(0);
		}
		if (leftEncoderValue >= count)
		{
			motors.setLeftSpeed(0);
		}
	}
	resetEncoderCounters();
}

bool turnLeft(int angle) // turn from 0-360grad
{
	int count=getCountsForAngle(angle);
	resetEncoderCounters();
	int speed = 75;
	motors.setSpeeds(-1 * (speed + 3), speed); //backwards is a little bit slower than forward
	while ((leftEncoderValue > -count) || (rightEncoderValue < count))
	{
		if (rightEncoderValue >= count)
		{
			motors.setRightSpeed(0);
		}
		if (leftEncoderValue <= -count)
		{
			motors.setLeftSpeed(0);
		}
	}
	motors.setSpeeds(0, 0);
	resetEncoderCounters();

}

bool turnRight(int angle) //turn from 0-360 grad
{
	int count = getCountsForAngle(angle);
	resetEncoderCounters();
	int speed = 75;
	motors.setSpeeds(speed+2, -1 * (speed));
	while ((rightEncoderValue > -count) || (leftEncoderValue < count))
	{
		if (leftEncoderValue >= count)
		{
			motors.setLeftSpeed(0);
		}
		if (rightEncoderValue <= -count)
		{
			motors.setRightSpeed(0);
		}
	}
	motors.setSpeeds(0, 0);
	resetEncoderCounters();
}

int getCountsForAngle(int angle)
{
	float countsF = COUNT_FULL_LEFT_CIRCLE * ((float)angle / 360);
	return (int)(countsF + 0.5);
}

void resetEncoderCounters()
{
	leftEncoderValue = 0;
	rightEncoderValue = 0;
}

int getCountsForDistance(int distanceInMM)
{
	float unrounded= (float)(((float)distanceInMM * (float)COUNTS_PER_CPOMPLETE_REVOLUTION) / (float)WHEEL_CIRCUMFERENCE);
	int roundedCount = (int)(unrounded + 0.5); //kaufmännisch gerundet
	return roundedCount;
}

void establishContact() {
	int i = 0;
	while (Serial.available() <= 0) {
		Serial.print("Serial.available: ");
		Serial.println(Serial.available());
		Serial.print("Arduino send: ");
		Serial.println(i);  //Print increasing value to Computer
		i += 1;
		delay(500);
	}
}