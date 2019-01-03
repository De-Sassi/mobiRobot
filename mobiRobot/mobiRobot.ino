/*
 Name:		mobiRobot.ino
 Created:	12/5/2018 3:21:29 PM
 Author:	delia
*/

#include <DFRobot_LCD.h>

#include <ZumoShield.h>

//Startbutton
#define PUSHBUTTON 19

//PINS
#define LED_PIN 13
#define LEFT_GREEN_PIN 25 
#define LEFT_YELLOW_PIN 50  //PCINT3 Do not change because of interrupts!
#define RIGHT_GREEN_PIN 24
#define RIGHT_YELLOW_PIN 51 //PCINT2 Do not change because of interrupts!
//
#define COUNTS_PER_CPOMPLETE_REVOLUTION 693
#define WHEEL_CIRCUMFERENCE 119 //in mm
#define COUNT_FULL_LEFT_CIRCLE 1545 

//for interrupt
#define cbi(sfr,bit) (_SFR_BYTE(sfr)&= ~_BV(bit)) //clear bit in byte at sfr adress
#define sbi(sfr,bit) (_SFR_BYTE(sfr)|= _BV(bit)) //Set bit in byte at sfr adress


ZumoMotors motors;

DFRobot_LCD lcd(16, 2);  //16 characters and 2 lines of show


ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
Pushbutton button(ZUMO_BUTTON);

//Encoder
volatile int leftReadingAPhase = LOW;
volatile int leftReadingAPhaseOld = LOW;
volatile int rightReadingAPhase = LOW;
volatile int rightReadingAPhaseOld = LOW;
volatile int leftEncoderValue = 0;
volatile int rightEncoderValue = 0;
volatile byte oldPortB;
volatile bool testEventFlag = false;


//Variable für Programmstart nicht gut!!!! interrupt falsch verwendet
volatile bool Prog_START = false;

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
	//attachInterrupt(digitalPinToInterrupt(LEFT_YELLOW_PIN), leftEncoder, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(RIGHT_YELLOW_PIN), rightEncoder, CHANGE);
	// ISCn1=0 und ISCn0=1 is for CHANGE
	oldPortB = PINB;
	digitalWrite(LEFT_YELLOW_PIN, HIGH);
	digitalWrite(RIGHT_YELLOW_PIN, HIGH);
	sbi(PCICR, PCIE0); //enable interrupt 
	sbi(PCMSK0, PCINT3); //enable interrupt for pin 50
	sbi(PCMSK0, PCINT2); //enable interrupt for pin 51

	//Pin und Interrupt Definitionen
	pinMode(PUSHBUTTON, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PUSHBUTTON), Pushbutton_change, RISING);
	// uncomment one or both of the following lines if your motors' directions need to be flipped
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);
	
	//line array
	reflectanceSensors.init();
}


void loop()
{


	if (Prog_START)
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("start");
		turnRight(30);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("end loop");
		delay(1500);
	}
	else
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Wait");
		delay(500);
	}




	//if (testEventFlag)
	//{
	//	Serial.print("Left Encoder: ");
	//	Serial.println(leftEncoderValue);
	//	Serial.print("right Encoder: ");
	//	Serial.println(rightEncoderValue);
	//	lcd.clear();
	//	lcd.println("eventTriggered");
	//	lcd.println(oldPortB, BIN);*/
	//	testEventFlag = false;
	//}

}


ISR(PCINT0_vect)
{
	const int rightBit = 3;
	const int leftBit = 2;
	cli(); //stop interrupts fro happening befor pin read;
	byte interruptVector = PINB;
	byte compareByte = interruptVector ^ oldPortB;
	////////////////////
	if ((compareByte >> rightBit) & 0x01)
	{
		//gets only the bit of the righ yellow pin
		byte onlyRightYellowBit = (interruptVector >> rightBit) & 0x01;
		rightEncoder(onlyRightYellowBit);
	}
	if ((compareByte >> leftBit) & 0x01)
	{
		//gets only the bit of the left yellow pin
		byte onlyLeftYellowBit = (interruptVector >> leftBit) & 0x01;
		leftEncoder(onlyLeftYellowBit);
	}
	
	testEventFlag = true;
	oldPortB = interruptVector; //for detecting the change
	sei(); //restart interrupts
}

void leftEncoder(byte yellow_pin_bit)
{
	leftReadingAPhase = digitalRead(LEFT_GREEN_PIN);
	if ((leftReadingAPhase == HIGH) && (leftReadingAPhaseOld == LOW))
	{
		if (yellow_pin_bit == HIGH)
		{
			leftEncoderValue++;
		}
		else {
			leftEncoderValue--;
		}
	}
	leftReadingAPhaseOld = leftReadingAPhase;
}

void rightEncoder(byte yellow_pin_bit)
{

	rightReadingAPhase = digitalRead(RIGHT_GREEN_PIN);
	if ((rightReadingAPhase == HIGH) && (rightReadingAPhaseOld == LOW))
	{
		if (yellow_pin_bit == HIGH)
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
	int speed = 100;
	motors.setSpeeds(-1 * (speed + 3), speed); //backwards is a little bit slower than forward
	int i = 0;
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
		i++;
		if (i % 20)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(rightEncoderValue);
			lcd.setCursor(6, 0);
			lcd.print(leftEncoderValue);
		}
	}
	motors.setSpeeds(0, 0);
	resetEncoderCounters();

}

bool turnRight(int angle) //turn from 0-360 grad
{
	int count = getCountsForAngle(angle);
	resetEncoderCounters();
	int speed = 100;
	motors.setSpeeds(speed+2, -1 * (speed));
	int i = 0;
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
		i++;
		if (i % 20)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(rightEncoderValue);
			lcd.setCursor(6, 0);
			lcd.print(leftEncoderValue);
		}
	}
	motors.setSpeeds(0, 0);
	resetEncoderCounters();
}

void Pushbutton_change() {
	Prog_START = !Prog_START;
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