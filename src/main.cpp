/****************************************************************

Hardware Connections:

IMPORTANT: The APDS-9960 can only accept 3.3V!
 
 ESP32 Pin  APDS-9960 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 D21          SDA              I2C Data
 D22          SCL              I2C Clock
 25           INT              Interrupt

LCD:
V0 - RESISTOR PARA O VSS -
 ESP32		LCD			Supply
 GND		Vss			---
 ---		Vdd			+5V




****************************************************************/

#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "LiquidCrystal.h"

// Pins
#define APDS9960_INT 	25 // Interrupt pin

#define PINO_TRIG 		4  // Pino D4 conectado ao TRIG do HC-SR04
#define PINO_ECHO 		2  // Pino D2 conectado ao ECHO do HC-SR04

#define DC1_INPUT1		27
#define DC1_INPUT2		26
#define DC1_ENABLE		14		
#define DC2_INPUT3		33
#define DC2_INPUT4		32
#define DC2_ENABLE		13

// Define PWM properties
const int freq = 18000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 10;
int dutyCicle = 800;

void interruptRoutine();
void handleGesture();

void rotate_left();
void rotate_right();
void forward();
void stop();

float get_distance();

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;
int commandSequence[50] = {0};
int qntGest = 0;
int cursorLCD = 0;
int startFLAG = 0, endFLAG = 0;
int currentGesture = 0;
int rotate_flag = 0;

// Initializing LCD
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

void setup() {

    // Set interrupt pin as input
    pinMode(APDS9960_INT, INPUT);
    
    // Set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

    // Print a message to the LCD.
    lcd.print("Start with NEAR");
	lcd.setCursor(0,1);
	lcd.print("    gesture    ");

    // Initialize Serial port
    Serial.begin(9600);
    Serial.println();
    Serial.println(F("--------------------------------"));
    Serial.println(F("SparkFun APDS-9960 - GestureTest"));
    Serial.println(F("--------------------------------"));

	// HC-SR04 Pins
	pinMode(PINO_TRIG, OUTPUT); // Configura o pino TRIG como saída
  	pinMode(PINO_ECHO, INPUT); // Configura o pino ECHO como entrada

	pinMode(DC1_INPUT1, OUTPUT);
    pinMode(DC1_INPUT2, OUTPUT);
    pinMode(DC1_ENABLE, OUTPUT);
    pinMode(DC2_INPUT3, OUTPUT);
    pinMode(DC2_INPUT4, OUTPUT);
    pinMode(DC2_ENABLE, OUTPUT);

	// configure LEDC PWM
	ledcSetup(pwmChannelA,freq,resolution);
	ledcSetup(pwmChannelB,freq,resolution);

	// Attach the PWM channels to the pins
 	ledcAttachPin(DC1_ENABLE, pwmChannelA);
  	ledcAttachPin(DC2_ENABLE, pwmChannelB);

	digitalWrite(DC2_INPUT3, LOW);
	digitalWrite(DC2_INPUT4, LOW);

    // Initialize interrupt service routine
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

    // Initialize APDS-9960 (configure I2C and initial values)
    if(apds.init())
        Serial.println(F("APDS-9960 initialization complete"));
	else
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    
    // Start running the APDS-9960 gesture sensor engine
    if(apds.enableGestureSensor(true))
        Serial.println(F("Gesture sensor is now running"));
	else 
        Serial.println(F("Something went wrong during gesture sensor init!"));

	// Turn on the display:
    lcd.display();
}

void loop() {
	float distance = get_distance();
	Serial.println(distance);

    if((isr_flag) && !(endFLAG)) {
        detachInterrupt(APDS9960_INT);
        handleGesture();
        isr_flag = 0;
        attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
	if(endFLAG) {
		while(commandSequence[currentGesture] != 6) {

			distance = get_distance();
			// Serial.println(distance);
		
			if(distance < 10 || rotate_flag){
				rotate_flag = 0;
				stop();

				if(currentGesture == 0)
					delay(3000);
				else 
					delay(1000);
				
				currentGesture++;
				if(commandSequence[currentGesture] == 3) {
					forward();
				}
				else if(commandSequence[currentGesture] == 2) {
					rotate_right();
					rotate_flag = 1;
					delay(1000);
				}
				else if(commandSequence[currentGesture] == 1) {
					rotate_left();
					rotate_flag = 1;
					delay(1000);
				}
			}
		}

		if(distance < 15) 
			stop();

	}
}

float get_distance(){
    long sum = 0;
    int numReadings = 7;  // número de leituras para média
    for(int i = 0; i < numReadings; i++){
        digitalWrite(PINO_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PINO_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PINO_TRIG, LOW);

        long duracao = pulseIn(PINO_ECHO, HIGH);
        sum += duracao;
        delay(10);  // pequeno delay entre leituras
    }
    float mediaDuracao = sum / numReadings;
    float distancia = (mediaDuracao * 0.0343) / 2;
    return distancia;
}


void rotate_right() {

	// Move Motor A forward
	digitalWrite(DC1_INPUT1, HIGH);
	digitalWrite(DC1_INPUT2, LOW);
	ledcWrite(pwmChannelA, dutyCicle);  // Set speed (0-255)

	// Move Motor B backward
	digitalWrite(DC2_INPUT3, LOW);
	digitalWrite(DC2_INPUT4, HIGH);
	ledcWrite(pwmChannelB, dutyCicle);  // Set speed (0-255)
	
	delay(300);

	// Stop both motors
	ledcWrite(pwmChannelA, 0);
	ledcWrite(pwmChannelB, 0);
	delay(1000);

}

void rotate_left() {
	// Move Motor A backward
	digitalWrite(DC1_INPUT1, LOW);
	digitalWrite(DC1_INPUT2, HIGH);
	ledcWrite(pwmChannelA, dutyCicle);  // Set speed (0-255)

	// Move Motor B forward
	digitalWrite(DC2_INPUT3, HIGH);
	digitalWrite(DC2_INPUT4, LOW);
	ledcWrite(pwmChannelB, dutyCicle);  // Set speed (0-255)
	
	delay(300);

	// Stop both motors
	ledcWrite(pwmChannelA, 0);
	ledcWrite(pwmChannelB, 0);
	delay(1000);
}

void forward() {
	// Move Motor A forward
	digitalWrite(DC1_INPUT1, LOW);
	digitalWrite(DC1_INPUT2, HIGH);
	ledcWrite(pwmChannelA, dutyCicle);  // Set speed (0-255)

	// Move Motor B backward
	digitalWrite(DC2_INPUT3, LOW);
	digitalWrite(DC2_INPUT4, HIGH);
	ledcWrite(pwmChannelB, dutyCicle);  // Set speed (0-255)
}

void stop(){
	// Stop both motors
	ledcWrite(pwmChannelA, 0);
	ledcWrite(pwmChannelB, 0);

}

void interruptRoutine() {
    isr_flag = 1;
}

void handleGesture() {
    if(apds.isGestureAvailable()){
		int gesture = apds.readGesture();
		Serial.println(gesture);
        if(gesture == DIR_UP && startFLAG == 1 && endFLAG == 0){
			commandSequence[qntGest] = 3;
			qntGest++;

			lcd.setCursor((cursorLCD % 16),(cursorLCD / 16));
			lcd.print("^");
			cursorLCD++;
			lcd.print(",");
			cursorLCD++;
		}
		else if(gesture == DIR_RIGHT && startFLAG == 1 && endFLAG == 0) {
			commandSequence[qntGest] = 2;
			qntGest++;

			lcd.setCursor((cursorLCD % 16),(cursorLCD / 16));
			lcd.print(">");
			cursorLCD++;
			lcd.print(",");
			cursorLCD++;
		}
		else if(gesture == DIR_LEFT && startFLAG == 1 && endFLAG == 0){
			commandSequence[qntGest] = 1;
			qntGest++;

			lcd.setCursor((cursorLCD % 16),(cursorLCD / 16));
			lcd.print("<");
			cursorLCD++;
			lcd.print(",");
			cursorLCD++;
		}
		else if(gesture == DIR_DOWN && startFLAG == 1 && endFLAG == 0 && qntGest > 1){
			qntGest--;
			commandSequence[qntGest] = 0;

			cursorLCD--;
			lcd.setCursor((cursorLCD % 16),(cursorLCD / 16));
			lcd.print(" ");
			cursorLCD--;
			lcd.setCursor((cursorLCD % 16),(cursorLCD / 16));
			lcd.print(" ");
		}
		else if(gesture == DIR_NEAR && startFLAG == 0 && endFLAG == 0){
			lcd.clear();
			commandSequence[qntGest] = 5;

			qntGest++;
			startFLAG = 1;
			lcd.setCursor(0,0);  
			lcd.print("{");
			cursorLCD++;
		}
		else if((gesture == DIR_FAR) && (startFLAG == 1) && (endFLAG == 0) && qntGest > 1){
			commandSequence[qntGest] = 6;
			qntGest++;

			endFLAG = 1;
			lcd.setCursor(((cursorLCD - 1) % 16),((cursorLCD - 1) / 16));
			lcd.print("}");
		}
    }
}