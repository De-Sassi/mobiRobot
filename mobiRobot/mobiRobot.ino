/*
 Name:		mobiRobot.ino
 Created:	12/5/2018 3:21:29 PM
 Author:	delia
*/

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#include <DFRobot_LCD.h>
#include <ZumoShield.h>

//Startbutton
#define PUSHBUTTON 19

//PINS Encoder
#define LED_PIN 13
#define LEFT_GREEN_PIN 25 
#define LEFT_YELLOW_PIN 50  //PCINT3 Do not change because of interrupts!
#define RIGHT_GREEN_PIN 24
#define RIGHT_YELLOW_PIN 51 //PCINT2 Do not change because of interrupts!
//Wheels
#define COUNTS_PER_CPOMPLETE_REVOLUTION 693
#define WHEEL_CIRCUMFERENCE 119 //in mm
#define COUNT_FULL_LEFT_CIRCLE 1545 

//for interrupt
#define cbi(sfr,bit) (_SFR_BYTE(sfr)&= ~_BV(bit)) //clear bit in byte at sfr adress
#define sbi(sfr,bit) (_SFR_BYTE(sfr)|= _BV(bit)) //Set bit in byte at sfr adress

//PINS Distance Sensors
#define RIGHT_DISTANCE_SENSOR 7
#define LEFT_DISTANCE_SENSOR 8
#define RIGHTFRONT_DISTANCE_SENSOR 9
#define LEFTFRONT_DISTANCE_SENSOR 10


ZumoMotors motors;
DFRobot_LCD lcd(16, 2);  //16 characters and 2 lines of show
ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
Pushbutton button(ZUMO_BUTTON);

PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);

//Encoder
volatile int leftReadingAPhase = LOW;
volatile int leftReadingAPhaseOld = LOW;
volatile int rightReadingAPhase = LOW;
volatile int rightReadingAPhaseOld = LOW;
volatile int leftEncoderValue = 0;
volatile int rightEncoderValue = 0;
volatile byte oldPortB;
volatile bool testEventFlag = false;

//Button
volatile bool Prog_START = false;

//Morse
uint8_t state = 0;
char Wort[6];
char Buchstabe[6];
int counterBuchstabe = 0;
int counterWort = 0;
char morse_chars[37] = { "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890" };
char morse_strings[37][6] = {
 ".-",
 "-...",
 "-.-.",
 "-..",
 ".",
 "..-.",
 "--.",
 "....",
 "..",
 ".---",
 "-.-",
 ".-..",
 "--",
 "-.",
 "---",
 ".--.",
 "--.-",
 ".-.",
 "...",
 "-",
 "..-",
 "...-",
 ".--",
 "-..-",
 "-.--",
 "--..",
 "-----",
 ".----",
 "..---",
 "...--",
 "....-",
 ".....",
 "-....",
 "--...",
 "---..",
 "----.", };
const int threshold = 700;



void setup()
{
	//Start communication
	//Serial.begin(9600);
	//while (!Serial)
	//{
	//	;
	//}
	//establishContact();
	//Serial.println("Serial connection ready");


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
	initLineArray();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("ready to start");

	//NFC
	nfc.begin();
}

void loop()
{
	driveOnLine();
	
	//unsigned int sensors[5];
	//reflectanceSensors.read(sensors); //Sensoren werden ausgelesen

	//lcd.clear();
	//lcd.setCursor(0, 0);
	//lcd.print(sensors[0]);
	//lcd.setCursor(4, 0);
	//lcd.print(sensors[1]);
	//lcd.setCursor(9, 0);
	//lcd.print(sensors[2]);
	//lcd.setCursor(0, 1);
	//lcd.print(sensors[3]);
	//lcd.setCursor(4, 1);
	//lcd.print(sensors[4]);
	//lcd.setCursor(9, 1);
	//lcd.print(sensors[5]);


	//bool line=lineDetected();
	//lcd.setCursor(15, 1);
	//lcd.print(line);
	//delay(2000);

	//if (Prog_START)
	//{
	//	lcd.clear();
	//	lcd.setCursor(0, 0);
	//	lcd.print("start");
	//	//////////


	//	/*driveToStraightBridge();

	//	lcd.clear();
	//	lcd.setCursor(0, 0);
	//	lcd.print("check bridge");
	//	delay(1000);*/
	//	bool stillOnLine = true;
	//	bool bridgeFree = distanceSensorShort() > 6;
	//	int i = 0;
	//	while (bridgeFree && stillOnLine)
	//	{
	//		if (i == 0)
	//		{
	//			lcd.clear();
	//			lcd.setCursor(0, 0);
	//			lcd.print("first while loop");
	//		}
	//		driveOnLine();
	//		stillOnLine = lineDetected();
	//		bridgeFree = distanceSensorShort() > 6;
	//		i++;

	//	}
	//	lcd.clear();
	//	lcd.setCursor(0, 0);
	//	lcd.print("out of while");
	//	motors.setSpeeds(0, 0);
	//	if (!bridgeFree)
	//	{
	//		lcd.clear();
	//		lcd.setCursor(0, 0);
	//		lcd.print("bridge is blocked");
	//	}
	//	else
	//	{
	//		if (!stillOnLine)
	//		{
	//			lcd.clear();
	//			lcd.setCursor(0, 0);
	//			lcd.print("bridge end");
	//		}
	//		else
	//		{

	//			lcd.clear();
	//			lcd.setCursor(0, 0);
	//			lcd.print("wtf??");
	//		}
	//	}
	//	delay(2000);
	//	///////////
	//	lcd.clear();
	//	lcd.setCursor(0, 0);
	//	lcd.print("end loop");
	//	delay(1500);
	//}
	//else
	//{
	//	motors.setSpeeds(0, 0);
	//	lcd.clear();
	//	lcd.setCursor(0, 0);
	//	lcd.print("Wait");
	//	delay(500);
	//}

}

/////////////////////////////
// modules

void driveToStraightBridge()
{
	turnLeft(30);
	driveDistance(450);
	turnRight(30);
	driveDistance(150);
	//start looking for line
	while (!lineDetected())
	{
		driveDistance(10);
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("line detected");

	delay(2000);
}

void driveOverStraightBridge()
{
	bool stillOnLine = true;
	while (stillOnLine)
	{
		driveOnLine();
		stillOnLine = lineDetected();
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Bridge end");
	delay(2000);

}


void driveToBridge1()
{
	turnRight(30);
	driveDistance(250);
	turnLeft(30);
	driveDistance(200);
	//start looking for line
	while (!lineDetected())
	{
		driveDistance(10);
	}

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("line detected");

	delay(2000);
}

void driveOnBridge1()
{
	//does not work anymore. has a problem with the line sensor. goes directly into bridge open

	bool stillOnLine = true;
	int i = 0;
	while ((distanceSensorShort() > 6) && stillOnLine)
	{
		driveOnLine();
		stillOnLine = lineDetected();
		if (i % 20)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(stillOnLine);
		}
		i++;

	}
	if (stillOnLine)
	{
		motors.setSpeeds(0, 0);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("road blocked");
		delay(2000);
	}
	else
	{
		//ready for bridge	
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("bridge open");
		motors.setSpeeds(0, 0);
		delay(2000);
		while (!lineDetected())
		{
			driverOverBridge();
		}
	}


}
/////////////
ISR(PCINT0_vect)
{
	const int rightBit = 3;
	const int leftBit = 2;

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

}

///////////////// 
//Drive

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
	int count = getCountsForDistance(distanceInMM);
	resetEncoderCounters();
	motors.setSpeeds(100, 100);
	bool leftMotorRunning = true;
	bool rightMotorRunning = true;
	while (rightMotorRunning || leftMotorRunning) //both motors need to be stopped to continue
	{
		cli();
		bool reachedRightLimit = rightEncoderValue >= count;
		bool reachedLeftLimit = leftEncoderValue >= count;
		sei();
		if (reachedRightLimit)
		{
			motors.setRightSpeed(0);
			rightMotorRunning = false;
		}
		if (reachedLeftLimit)
		{
			motors.setLeftSpeed(0);
			leftMotorRunning = false;
		}
	}
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(rightEncoderValue);
	lcd.setCursor(6, 0);
	lcd.print(leftEncoderValue);
	resetEncoderCounters();
}

bool turnLeft(int angle) // turn from 0-360grad
{
	int count = 200;// getCountsForAngle(angle);
	resetEncoderCounters();
	int speed = 100;
	motors.setSpeeds(-1 * speed, speed);
	bool leftMotorRunning = true;
	bool rightMotorRunning = true;
	while (rightMotorRunning || leftMotorRunning)
	{
		cli();
		bool reachedRightLimit = rightEncoderValue >= count;
		bool reachedLeftLimit = leftEncoderValue <= -1 * count;
		sei();
		if (reachedRightLimit)
		{
			motors.setRightSpeed(0);
			rightMotorRunning = false;
		}
		if (reachedLeftLimit)
		{
			motors.setLeftSpeed(0);
			leftMotorRunning = false;
		}


	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(rightEncoderValue);
	lcd.setCursor(6, 0);
	lcd.print(leftEncoderValue);
	resetEncoderCounters();

}

bool turnRight(int angle) //turn from 0-360 grad
{
	int count = 200;//getCountsForAngle(angle);
	resetEncoderCounters();
	int speed = 100;
	motors.setSpeeds(speed, -1 * speed);
	bool leftMotorRunning = true;
	bool rightMotorRunning = true;
	while (rightMotorRunning || leftMotorRunning)
	{
		cli();
		bool reachedRightLimit = rightEncoderValue <= -1 * count;
		bool reachedLeftLimit = leftEncoderValue >= count;
		sei();
		if (reachedLeftLimit)
		{
			motors.setLeftSpeed(0);
			leftMotorRunning = false;
		}
		if (reachedRightLimit)
		{
			motors.setRightSpeed(0);
			rightMotorRunning = false;
		}


	}
	motors.setSpeeds(0, 0);

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(rightEncoderValue);
	lcd.setCursor(6, 0);
	lcd.print(leftEncoderValue);

	resetEncoderCounters();
}

int getCountsForAngle(int angle)
{
	float countsF = COUNT_FULL_LEFT_CIRCLE * ((float)angle / 360);
	return (int)(countsF + 0.5);
}

void resetEncoderCounters()
{
	cli();
	leftEncoderValue = 0;
	rightEncoderValue = 0;
	sei();
}

int getCountsForDistance(int distanceInMM)
{
	float unrounded = (float)(((float)distanceInMM * (float)COUNTS_PER_CPOMPLETE_REVOLUTION) / (float)WHEEL_CIRCUMFERENCE);
	int roundedCount = (int)(unrounded + 0.5); //kaufmännisch gerundet
	return roundedCount;
}
///////////////////////////////////////

void Pushbutton_change() {
	Prog_START = !Prog_START;
	//not right yet. Button is not debounced yet
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


//////////////////////
//Line Array
void initLineArray()
{

	//sensors[0] (leftest) is broken;
	//sensor[5] is most right

	bool calibrateWithTurn = true;
	if (calibrateWithTurn)
	{
		// Initialize the reflectance sensors module
		reflectanceSensors.init();

		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Press sButton");

		// Wait for the user button to be pressed and released
		button.waitForButton();

		// Wait 1 second and then begin automatic sensor calibration
	// by rotating in place to sweep the sensors over the line
		delay(1000);
		int i;
		for (i = 0; i < 80; i++)
		{
			if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
				motors.setSpeeds(-200, 200);
			else
				motors.setSpeeds(200, -200);
			reflectanceSensors.calibrate();

			// Since our counter runs to 80, the total delay will be
			// 80*30 = 2400 ms.
			delay(30);
		}
		motors.setSpeeds(0, 0);
	}
	else
	{
		reflectanceSensors.init();
		reflectanceSensors.calibrate();
	}
}

void driveOnLine()
{
	unsigned int sensors[5];
	const int MAX_SPEED = 100; //Das Programm ist für diesen Speed ausgelegt
	reflectanceSensors.read(sensors); //Sensoren werden ausgelesen

	int speedDifference = lineposition(sensors);
	Serial.println(speedDifference);
	int m1Speed = MAX_SPEED - speedDifference;
	int m2Speed = MAX_SPEED + speedDifference;
	motors.setSpeeds(m1Speed, m2Speed); //negative Geschwindigkeit für Vorwärts
}

void readMorseCode()
{
	bool endMorse = false;
	while (!endMorse)
	{
		endMorse = readMorse();
		delay(10); //otherwise to fast. cant read all the letters. why? dont know..
	}
	delay(2000);

}

bool readMorse()
{
	motors.setSpeeds(80, 80);

	int blackDistance = 0;
	int whiteDistance = 0;

	const int anfahren = 0;
	const int white = 1;
	const int black = 2;
	const int newLetter = 3;
	const int wordEnd = 4;
	const int digitalIRSensor = 23;

	unsigned int sensors[5];
	reflectanceSensors.read(sensors); //Sensoren werden ausgelesen


	switch (state) {
	case anfahren: //Anfahren
		if (digitalRead(digitalIRSensor) == false) {
			state = black;
			resetEncoderCounters();
		}

		else { state = anfahren; };
		break;

	case white: //white 
		whiteDistance = getDistanceForCounts(rightEncoderValue);

		if (digitalRead(digitalIRSensor) == true) { state = 1; digitalWrite(LED_BUILTIN, HIGH); }
		else {
			whiteDistance = getDistanceForCounts(rightEncoderValue);
			resetEncoderCounters();
			//Serial.println(whiteDistance);
			if (whiteDistance <= 6) { state = black; }
			else { state = newLetter; }
		}
		if (whiteDistance > 18) {
			state = newLetter;//Serial.println("state1zu3");
		}

		break;

	case black: //black 
		if (digitalRead(digitalIRSensor) == false) { state = 2; digitalWrite(LED_BUILTIN, LOW); }
		else {
			blackDistance = getDistanceForCounts(rightEncoderValue);
			resetEncoderCounters();
			//Serial.println("black" + blackDistance);
			if (blackDistance < 6) { Buchstabe[counterBuchstabe] = '.'; }//falls 5mm = Punkt sonst Strich
			else { Buchstabe[counterBuchstabe] = '-'; };
			//Serial.println(Buchstabe);
			++counterBuchstabe;
			state = white;
		}
		break;

	case newLetter: //new Buchstabe
	  //Serial.println("case 3");
		digitalWrite(LED_BUILTIN, HIGH);
		for (int i = 0; i < 36; ++i) {
			if (strcmp(Buchstabe, morse_strings[i]) == 0) {
				//Serial.println(morse_strings[i]);
				Wort[counterWort] = morse_chars[i];
				//Serial.println(morse_chars[i]);
			};
		};
		++counterWort;
		//Serial.println(counterWort);
		counterBuchstabe = 0;
		for (int i = 0; i < 6; ++i) {
			Buchstabe[i] = { 0 };
		}
		//Serial.println(whiteDistance);
		if (getDistanceForCounts(rightEncoderValue) > 18) { state = wordEnd; }
		else { state = black; }
		break;

	case wordEnd: //Wort zu ende
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(Wort);
		motors.setSpeeds(0, 0);
		delay(2000);
		for (int i = 0; i < 6; ++i) {
			Wort[i] = { 0 };
		}
		state = anfahren;
		counterWort = 0;
		return true;
		break;
	}
	return false;
}

int getDistanceForCounts(int counts)
{

	float distanceInMM = (float)(((float)counts *(float)WHEEL_CIRCUMFERENCE) / (float)COUNTS_PER_CPOMPLETE_REVOLUTION);
	int returnWert = (int)distanceInMM;

	return returnWert;
}

int lineposition(unsigned int sensors[5]) {
	int Position = 0;
	if (sensors[1] > threshold) { Position += 40; }
	else { Position -= 40; };
	if (sensors[2] > threshold) { Position += 20; }
	else { Position -= 10; };
	if (sensors[3] > threshold) { Position -= 20; }
	else { Position += 20; };
	if (sensors[4] > threshold) { Position -= 40; }
	else { Position += 40; };
	return Position;
}

bool lineDetected() {
	unsigned int sensors[5];
	reflectanceSensors.read(sensors);
	if (sensors[1] > threshold || sensors[2] > threshold || sensors[3] > threshold || sensors[4] > threshold) {
		return true;
	}
	else {
		return false;
	}
}

///////////////////
//Banden Fahren
void driverOverBridge()
{
	//rechter Sensor A13
	//linker Sensor A14
	const int MAX_SPEED = 100; //Das Programm ist für diesen Speed ausgelegt
	int speedDifference = (distanceNormalSensor(RIGHT_DISTANCE_SENSOR) - distanceNormalSensor(LEFT_DISTANCE_SENSOR)) * 20;
	int m1Speed = MAX_SPEED + speedDifference;
	int m2Speed = MAX_SPEED - speedDifference;
	motors.setSpeeds(m1Speed, m2Speed);
}

//////////////////////
//NFC tag
void readNFCTag()
{
	if (nfc.tagPresent()) {
		NfcTag tag = nfc.read();
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(tag.getNdefMessage().getRecord(0).getText());
	}
	delay(4000);
	lcd.clear();
}

/////////////////////
//Distance Sensors

///Formel für Sharp Sensor in cm GP2Y0A41 (4-30cm)
double distanceNormalSensor(int analogPinOfSensor)
{
	return 2241.2*pow(analogRead(analogPinOfSensor), -0.96) - 1;
}
/// SHARP GP2Y0A51SK0F (2 - 15cm)
double distanceSensorShort()
{
	int analogSensorPin = 6;
	return (783.35*pow(analogRead(analogSensorPin), -0.841)*0.8 - 1.8);
}