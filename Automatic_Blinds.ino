/*
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 */
 
#include <Stepper.h>

// change this to fit the number of steps per revolution
// for your motor
const int stepsPerRevolution = 2036;
// the number of the pushbutton pin
const int buttonPin = 2;
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 10, 11, 12, 13);
// variable for reading the pushbutton status
int buttonState = 0;
// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int on = 0; //This tells the status 
void setup() {
  // set the speed at 9 rpm:
  myStepper.setSpeed(9);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    do{
    Serial.println("shades down");
    myStepper.step(stepsPerRevolution);
    }while(sensorValue > 50);
    
  }
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);

  if(sensorValue > 50){
    // step one revolution  in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    //delay(500);
  }
  if(sensorValue < 50){
    // step one revolution in the other direction:
    Serial.println("counterclockwise");
    myStepper.step(-(stepsPerRevolution));
    //delay(500);
  }
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
