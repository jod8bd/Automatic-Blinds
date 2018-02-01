#include <Scheduler.h>
//The pin for each input to the stepper motor
#define IN1  10
#define IN2  11
#define IN3  12
#define IN4  13
int Steps = 0;              //Counts the number of times the motor goes through rotation
boolean Direction = true;   //Using boolean logic to determine the direction
unsigned long last_time;    //Holds the last time the loop completed
unsigned long currentMillis;//Holds current time of the loop
int steps_left=4095;        //How many steps for the step motor to take
long time;
const int analogInPin = A0; //Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; //Analog output pin that the LED is attached to
const int buttonPin = 8;
int sensorValue = 0;        //Value read from the pot
int outputValue = 0;        //Output to the PWM (analog out) used for testing purposes.
int buttonState = 0;        //Handles the state of the button
void setup()
{
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  delay(200);
  if (buttonState == HIGH) {
  while(1){
  while(steps_left>0)
  {
    currentMillis = micros();
    if(currentMillis-last_time>=1000)
    {
      stepper(1);
      time=time+micros()-last_time;
      last_time=micros();
      steps_left--;
    }
  }
  //Serial.println(time);
  Direction=!Direction;
  steps_left=2048;
  }
}
}

void stepper(int xw)
{
  for (int x=0;x<xw;x++)
  {
    switch(Steps)
    {
      case 0:
      //Serial.println("Here 0");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
      case 1:
      //Serial.println("Here 1");
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      break; 
      case 2:
      //Serial.println("Here 2");
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break; 
      case 3:
      //Serial.println("Here 3");
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
      case 4:
      //Serial.println("Here 4");
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break; 
      case 5:
      //Serial.println("Here 5");
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break; 
      case 6:
      //Serial.println("Here 6");
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break; 
      case 7:
      //Serial.println("Here 7");
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break; 
      default:
      //Serial.println("here");
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    }
    SetDirection();
  }
} 
void SetDirection()
{
  //Calls function that reads sensor and returns sensorValue.
  sensorValue = readSense();
  //Serial.println(sensorValue);
  //Conditional statement that checks for a low light ADC reading.
  if(sensorValue > 150){ Steps++;}
  if(sensorValue < 150){ Steps--; }
  //These set of conditional statements keep the motor from continuously
  //running in one direction.
  if(Steps>7){Steps=0;}
  if(Steps<0){Steps=7; }
}
int readSense()
{
  sensorValue = analogRead(analogInPin);
  //map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  //change the LED light level based on the rotation of the motor
  analogWrite(analogOutPin, outputValue);
  return sensorValue;
}
