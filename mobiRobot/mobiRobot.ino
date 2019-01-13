/*
 Name:		mobiRobot.ino
 Created:	12/5/2018 3:21:29 PM
 Author:	delia
*/


//#include <SharpDistSensor.h>
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#include <DFRobot_LCD.h>
#include <ZumoShield.h>
#include <IR_Thermometer_Sensor_MLX90614.h>

//Startbutton
#define PUSHBUTTON 19

//MAGNET
#define MAGNET_ANALOG A11

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

////PIN DistanceSensors with library
//#define RIGHT_DISTANCE_SENSOR_A A7
//#define LEFT_DISTANCE_SENSOR_A A8
//#define RIGHTFRONT_DISTANCE_SENSOR_A A9
//#define LEFTFRONT_DISTANCE_SENSOR_A A10
//#define SHORT_SENSOR_A A6

//MorseSensor
#define DIGITAL_IR_SENSOR 23

//IR SEnsor Array.
#define DO_NOT_USE_BECAUSE_IT_IS_SOLDERD 2
#define EMITTER_PIN A4
#define BROKEN_SENSOR0 4
#define SENSOR1 A3
#define SENSOR2 11  //ATTENTION! Is on interrupt vector. maybe turn off?
#define SENSOR3 A0
#define SENSOR4 A2
#define NOT_USED_SENSOR5 5

ZumoMotors motors;
DFRobot_LCD lcd(16, 2);  //16 characters and 2 lines of show
ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
Pushbutton button(ZUMO_BUTTON);
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
IR_Thermometer_Sensor_MLX90614 Temperatursensor = IR_Thermometer_Sensor_MLX90614();


//SharpDistSensor rightSensor(RIGHT_DISTANCE_SENSOR_A );
//SharpDistSensor leftSensor(LEFT_DISTANCE_SENSOR_A);
//SharpDistSensor frontRightSensor(RIGHTFRONT_DISTANCE_SENSOR_A);
//SharpDistSensor frontLeftSensor(LEFTFRONT_DISTANCE_SENSOR_A);
//SharpDistSensor shortSensor(SHORT_SENSOR_A);


//Programm general
const int generalMotorSpeed = 100;
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
//Line
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

	//set encoder pins
	//left
	pinMode(LEFT_GREEN_PIN, INPUT);
	pinMode(LEFT_YELLOW_PIN, INPUT);
	//right
	pinMode(RIGHT_GREEN_PIN, INPUT);
	pinMode(RIGHT_YELLOW_PIN, INPUT);

	//Initialize interupts
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

	//NFC
	nfc.begin();

	//Temperatur
	Temperatursensor.begin();

	////Sharp Sensors
	//rightSensor.setModel(rightSensor.GP2Y0A41SK0F_5V_DS);
	//leftSensor.setModel(leftSensor.GP2Y0A41SK0F_5V_DS);
	//frontRightSensor.setModel(frontRightSensor.GP2Y0A41SK0F_5V_DS);
	//frontLeftSensor.setModel(frontLeftSensor.GP2Y0A41SK0F_5V_DS);;
	//shortSensor.setModel(shortSensor.GP2Y0A51SK0F_5V_DS);

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("ready to start");
}


void loop()
{
	
	//driveOnLine();
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


	if (Prog_START)
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("start");
		//////////

		driveToFirstBridge();
		bool crossedBridge = driveOverFirstBridge();
		if (crossedBridge)
		{
			driveDistanceBack(170);
			turnRight(90);
			driveDistance(420);
			turnLeft(90);
			bool stillOnLine = true;
			while (stillOnLine) //follow the line until end
			{

				driveOnLine();
				stillOnLine = lineDetected();
			}
			motors.setSpeeds(0, 0);

			/*	firstNFCTag_FirstBridge();
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("read Tag");

				driveToMagnet1_FirstBridge();
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Magnet measure");
				delay(1000);
				measureMagnetfield();
				delay(1000);*/
				//driveToHeat1StraightBridge();
		}
		else
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("go to other bridge");
			turnRight(85);
			driveDistance(500); //drive to Other Bridge
			driveOverSecondBridge();
			/*strategySecondBridge();*/
		}
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("start strategy");
		delay(2000);
		strategySecondBridge(crossedBridge);

		delay(2000);

		/////////////
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("end loop");
		delay(1500);
	}
	else
	{
		motors.setSpeeds(0, 0);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Wait");
		delay(800);
	}

}

/////////////////////////////
// modules


void driveToHeat1StraightBridge() {
	driveDistanceBack(10);
	turnRight(50);
	driveDistance(190);
	readTemperature();
}

//////////
//crossed over second bridge

void strategySecondBridge(bool firstBridgeCrossed) {

	//ab nfc tag 1
	firstNFCTag();
	//read first temp
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Find Fire");
	turnRight(90);
	driveDistance(90);
	readTemperature();

	//go to second nfc tag
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Go to NFC-Tag");
	turnLeft(10);
	driveDistance(270);
	turnLeft(80);
	while (distanceNormalSensor(LEFTFRONT_DISTANCE_SENSOR) > 17)
	{
		driveOnRightSensorWithDistance(6);
	} //der Bande entlang fahren mit 6cm Abstand bis NFC TAg
	firstNFCTag();

	//go to magnet field
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Find Magnet 1");
	turnLeft(100);
	driveIntoPillar();
	measureMagnetfield();
	driveDistanceBack(40);

	//read secondTemp
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Find Fire");
	turnLeft(90);
	NeunziggradaufLinefahrenRight();
	bool wayfree = true;  //Gestrichelte Linie wurde angefahren und wird nun gefolgt bis poller abstand 6cm.
	while (wayfree)
	{
		driveOnLine();
		wayfree = distanceSensorShort() > 6;

	}
	turnRight(45);
	driveDistance(310);
	readTemperature();

	//read second magnet
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Find Magnet 2");
	driveDistance(80);
	turnLeft(45);
	driveIntoPillar();
	measureMagnetfield();
	driveDistanceBack(40);

	//read third nfc
	turnLeft(90);
	NeunziggradaufLinefahrenRight();


	//Bande erkennen
	wayfree = true;
	while (wayfree) {
		driveOnLine();
		//wayfree = frontLeftSensor.getDist() > 140;
		wayfree = distanceNormalSensor(LEFTFRONT_DISTANCE_SENSOR) > 14;
		//delay(50);
	}
	turnLeft(90);
	driveDistance(50);
	bool lineFound = false;
	motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
	while (!lineFound)
	{
		driveOnRightSensorWithDistance(22);
		lineFound = lineDetected();
		delay(50);
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("reached Bridge line");
	delay(1000);

	//drives forward to be right on line
	resetEncoderCounters();
	while (leftEncoderValue < 580)
	{
		driveOnLine();
	}
	driveDistanceBack(100);


	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("read Tag");
	bool tagRead = false;
	while (!tagRead)
	{
		driveDistanceBack(10);
		tagRead = readNFCTag();
	}
	delay(4000);

	driveDistance(50);
	if (firstBridgeCrossed)
	{
		/*GoBack to second bridge*/
		driveDistance(180);
		turnLeft(90);
		driveDistance(480);
		//Travel bridge
		driveOverSecondBridge();
		//go to first bridge
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("go to bridge one");
		delay(4000);
		driveDistanceBack(80);
		turnRight(90);
		driveDistance(450);
		NeunziggradaufLinefahrenLeft();


	}

	/*cross bridge*/

	bool stillOnLine = true;
	while (stillOnLine) //Should see the line and follow it
	{

		driveOnLine();
		stillOnLine = lineDetected();

	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("end straight bridge");
	delay(1000);

	//go to first morse code
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("drive to morse");
	turnRight(90);
	motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
	while (distanceSensorShort() > 3)
	{
		delay(100);
	}
	turnLeft(90);
	delay(1000);
	while (!lineDetected())
	{
		driveOnRightSensorWithDistance(7);
	}
	motors.setSpeeds(0, 0);
	driveDistance(110);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("read Morsecode");
	readMorseCode();

	////drive to 3 temp
	driveDistance(50);
	turnLeft(90);
	while (!lineDetected())
	{
		driveOnRightSensorWithDistance(10);
	}
	driveDistance(50);
	turnLeft(90);
	driveDistance(80);
	motors.setSpeeds(0, 0);
	turnRight(90);
	int ticks = getCountsForDistance(100);
	resetEncoderCounters();
	while (leftEncoderValue < ticks)
	{
		driveOnLine();
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("edge");
	turnLeft(45);
	driveDistance(180);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("read temp");
	delay(2000);
	lcd.clear();
	readTemperature();

	//next morse
	turnLeft(45);
	NeunziggradaufLinefahrenRight();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("on line");
	while (distanceNormalSensor(RIGHTFRONT_DISTANCE_SENSOR) > 6)
	{
		driveOnLine();
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("border");
	delay(1000);
	turnRight(180);
	driveDistanceBack(80);
	turnLeft(90);
	driveDistanceBack(50);
	int tick = getCountsForDistance(80);
	resetEncoderCounters();
	while (leftEncoderValue < tick)
	{
		driveOnLeftSensorWithDistance(5);
	}

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("ready to read code");
	delay(1000);
	readMorseCode();

	//drive to start
	turnRight(30);
	driveDistance(90);
	turnRight(60);
	driveDistance(80);
	while (!lineDetected())
	{
		driveOnLeftSensorWithDistance(10);
	}
	motors.setSpeeds(0, 0);
	driveDistance(100);
	turnRight(90);



}

void driveOverSecondBridge()
{
	bool stillOnLine = true;
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("drive on line");
	while (stillOnLine) //Should see the line and follow it
	{
		driveOnLine();
		stillOnLine = lineDetected();
	}
	motors.setSpeeds(0, 0);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("drive over bridge");
	delay(2000);
	stillOnLine = false;
	while (!stillOnLine) { //when line endet up, follow the Bridgebunches until sees next line 
		driverOverBridge();
		stillOnLine = lineDetected();
	}
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("drive on line");
	while (stillOnLine) //follow the line until end
	{

		driveOnLine();
		stillOnLine = lineDetected();
	}
	motors.setSpeeds(0, 0);
	//firstNFCTag(); //Read the NFC Tag !!!!Caution NFC Tag is broken ->no Output


}
////////
//crossed over first bridge


void driveToFirstBridge()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("go to straight bridge");
	turnLeft(45);
	driveDistance(480);
	turnRight(45);
	driveDistance(250);
	delay(1000);
}

bool driveOverFirstBridge()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("check bridge");
	delay(1000);
	bool stillOnLine = true;
	bool bridgeFree = distanceSensorShort() > 6;
	while (bridgeFree && stillOnLine)
	{
		driveOnLine();
		stillOnLine = lineDetected();
		bridgeFree = distanceSensorShort() > 6;
	}
	motors.setSpeeds(0, 0);
	bool isBridgeCrossed = false;
	if (!bridgeFree)
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("bridge is blocked");
	}
	else
	{
		if (!stillOnLine)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("bridge end");
			isBridgeCrossed = true;
		}
		else
		{

			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("wtf??");
			isBridgeCrossed = true; //not sure how this state should be handled. shoudl not be possible. never happend
		}
	}
	delay(2000);
	return isBridgeCrossed;
}

void driveToMagnet1_FirstBridge()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Magnet drive");
	delay(1000);
	driveDistance(150);
	turnRight(10);
	driveDistance(105);
}

void firstNFCTag()
{
	driveDistance(50);
	bool tagRead = false;
	while (!tagRead)
	{
		driveDistance(10);
		tagRead = readNFCTag();
	}
	delay(4000);
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

void turnLeft(int angle) // turn from 0-360grad
{
	int count = getCountsForAngle(angle);
	resetEncoderCounters();
	motors.setSpeeds(-1 * generalMotorSpeed, generalMotorSpeed);
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

void turnRight(int angle) //turn from 0-360 grad
{
	int count = getCountsForAngle(angle);
	resetEncoderCounters();
	motors.setSpeeds(generalMotorSpeed, -1 * generalMotorSpeed);
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

void NeunziggradaufLinefahrenRight() {
	bool linedetected = false;
	while (!linedetected) {
		motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
		linedetected = lineDetected();
	}
	motors.setSpeeds(0, 0);
	driveDistance(30);
	turnRight(90);
}
void NeunziggradaufLinefahrenLeft() {
	bool linedetected = false;
	while (!linedetected) {
		motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
		linedetected = lineDetected();
	}
	motors.setSpeeds(0, 0);
	driveDistance(30);
	turnLeft(90);
}

void driveIntoPillar()
{
	//drive into pillar
	double distance = distanceSensorShort();
	motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
	while (distance > 1.9)
	{
		distance = distanceSensorShort();
		delay(100);
	}
	motors.setSpeeds(0, 0);
	motors.setSpeeds(generalMotorSpeed, generalMotorSpeed);
	delay(1000);
	motors.setSpeeds(0, 0);

}

void driveDistanceBack(int distanceInMM)
{
	int count = getCountsForDistance(-distanceInMM);
	resetEncoderCounters();
	motors.setSpeeds(-100, -100);
	bool leftMotorRunning = true;
	bool rightMotorRunning = true;
	while (rightMotorRunning || leftMotorRunning) //both motors need to be stopped to continue
	{
		cli();
		bool reachedRightLimit = rightEncoderValue <= count;
		bool reachedLeftLimit = leftEncoderValue <= count;
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
	resetEncoderCounters();
}
///////////////////////////////////////

void Pushbutton_change() {
	Prog_START = !Prog_START;
	delay(10);
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
	bool calibrateWithTurn = true; //for debugging reasons
	pinMode(EMITTER_PIN, OUTPUT);
	//pinMode(DO_NOT_USE_BECAUSE_IT_IS_SOLDERD, OUTPUT);
	//digitalWrite(EMITTER_PIN,LOW);
	//digitalWrite(DO_NOT_USE_BECAUSE_IT_IS_SOLDERD, LOW);


	// Initialize the reflectance sensors module
	//byte pins[3] = { SENSOR1,SENSOR3,SENSOR4 };
	reflectanceSensors.init(EMITTER_PIN);
	//reflectanceSensors.init(pins,3,2000,EMITTER_PIN);
	if (calibrateWithTurn)
	{
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
		reflectanceSensors.calibrate();
	}
}

void driveOnLine()
{
	int speedDifference = lineposition();
	int m1Speed = generalMotorSpeed - speedDifference;
	int m2Speed = generalMotorSpeed + speedDifference;
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
	const int specialMorseMotorSpeed = 80;
	motors.setSpeeds(specialMorseMotorSpeed, specialMorseMotorSpeed);

	int blackDistance = 0;
	int whiteDistance = 0;

	const int anfahren = 0;
	const int white = 1;
	const int black = 2;
	const int newLetter = 3;
	const int wordEnd = 4;

	unsigned int sensors[5];
	reflectanceSensors.read(sensors); //Sensoren werden ausgelesen


	switch (state) {
	case anfahren: //Anfahren
		if (digitalRead(DIGITAL_IR_SENSOR) == false) {
			state = black;
			resetEncoderCounters();
		}

		else { state = anfahren; };
		break;

	case white: //white 
		whiteDistance = getDistanceForCounts(rightEncoderValue);

		if (digitalRead(DIGITAL_IR_SENSOR) == true) { state = 1; digitalWrite(LED_BUILTIN, HIGH); }
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
		if (digitalRead(DIGITAL_IR_SENSOR) == false) { state = 2; digitalWrite(LED_BUILTIN, LOW); }
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

int lineposition() {

	unsigned int sensors[5];
	reflectanceSensors.read(sensors);
	int Position = 0;
	int SensorsOn = 0;

	if (sensors[1] > threshold)
	{
		Position += 80;
		++SensorsOn;
	};
	if (sensors[2] > threshold)
	{
		Position += 40;
		++SensorsOn;
	};
	if (sensors[3] > threshold) {
		Position -= 40;
		++SensorsOn;
	};
	if (sensors[4] > threshold) {
		Position -= 80;
		++SensorsOn;
	};
	Position = Position / SensorsOn;
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
	int speedDifference = (distanceNormalSensor(RIGHT_DISTANCE_SENSOR) - distanceNormalSensor(LEFT_DISTANCE_SENSOR)) * 20;
	int m1Speed = generalMotorSpeed + speedDifference;
	int m2Speed = generalMotorSpeed - speedDifference;
	motors.setSpeeds(m1Speed, m2Speed);
}

void driveOnRightSensorWithDistance(int Distance)
{
	int distancebefore = distanceNormalSensor(RIGHT_DISTANCE_SENSOR);
	/*int distancebefore = rightSensor.getDist() / 10;*/
	int speedDifference = (Distance - distancebefore) * 15;
	if (speedDifference > 40) { speedDifference = 40; }
	if (speedDifference < -40) { speedDifference = -40; }

	lcd.print(speedDifference);
	lcd.clear();

	int m1Speed = generalMotorSpeed - speedDifference;
	int m2Speed = generalMotorSpeed + speedDifference;
	motors.setSpeeds(m1Speed, m2Speed);
}

void driveOnLeftSensorWithDistance(int Distance)
{
	int distancebefore = distanceNormalSensor(LEFT_DISTANCE_SENSOR);
	/*int distancebefore = rightSensor.getDist() / 10;*/
	int speedDifference = (Distance - distancebefore) * 15;
	if (speedDifference > 40) { speedDifference = 40; }
	if (speedDifference < -40) { speedDifference = -40; }

	lcd.print(speedDifference);
	lcd.clear();

	int m1Speed = generalMotorSpeed + speedDifference;
	int m2Speed = generalMotorSpeed - speedDifference;
	motors.setSpeeds(m1Speed, m2Speed);
}

//////////////////////
//NFC tag
bool readNFCTag()
{
	if (nfc.tagPresent()) {
		NfcTag tag = nfc.read();
		String text = tag.getNdefMessage().getRecord(0).getText();
		String rightSizeText = text.substring(0, 16);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(rightSizeText);
		return true;
	}
	return false;
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

///////////////
//Heat

void readTemperature() {
	long median;
	float ob_temp[5];
	for (int i = 0; i < 5; i++) {
		ob_temp[i] = Temperatursensor.GetObjectTemp_Celsius();
	}
	median = getMedian(5, ob_temp);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Temperatur");
	lcd.setCursor(0, 1);
	lcd.print(median);
	lcd.print("C");
	delay(2000);
}


////////////////////
//Magnet
void measureMagnetfield() {
	long median;
	double d[5];
	float a[5];

	//const int underLimitStrongMagnet = 504;
	//const int upperLimitStrongMagnet = 528;

	for (int i = 0; i < 5; ++i) { // Messung analog (5x in halber Sekunde)
		a[i] = analogRead(MAGNET_ANALOG); // *5.0 / 1023.0;
		delay(100);
	}

	median = getMedian(5, a); // Median der 5 Werte

	if (514 < median && median < 518) {
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("kein Magnet");
	}
	else {

		if (median < 504 || median > 528) { //ursprünglich 502 und 530
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("starkes Magnet");
		}
		else {
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("schwaches Magnet");
		}
	}
	delay(2000);
}


float getMedian(int n, float x[]) { // Medianwert bestimmen
	float temp;
	int i, j;

	// Sortierung des Arrays von grösstem zu kleinstem Wert
	for (i = 0; i < n - 1; i++) {
		for (j = i + 1; j < n; j++) {
			if (x[j] < x[i]) { // swap elements
				temp = x[i];
				x[i] = x[j];
				x[j] = temp;
			}
		}
	}
	// Ausgabe Medianwert
	if (n % 2 == 0) {
		return ((x[n / 2 - 1] + x[n / 2]) / 2);
	}
	else {
		return (x[n / 2 + (n % 2) / 2]);
	}
}