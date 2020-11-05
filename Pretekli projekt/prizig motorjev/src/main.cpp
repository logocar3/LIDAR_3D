#include <Arduino.h>

// desni motor
int MDP1 = 27; 
int MDP2 = 26; 
int enableDESNI = 14; 

// levi motor
int MLP3 = 34; 
int MLP4 = 12; 
int enableLEVI = 13; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelD = 1;
const int resolution = 8;
int dutyCycleL = 180;
int dutyCycleD = 180;

void setup() {
  Serial.begin(9600);
  Serial.println("test");
  pinMode(MLP3, OUTPUT);
  pinMode(MLP4, OUTPUT);
  pinMode(enableLEVI, OUTPUT);
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(enableLEVI, pwmChannelL);

  pinMode(MDP1, OUTPUT);
  pinMode(MDP2, OUTPUT);
  pinMode(enableDESNI, OUTPUT);
  ledcSetup(pwmChannelD, freq, resolution);
  ledcAttachPin(enableDESNI, pwmChannelD);
  
  Serial.println("testiramo  motor...");

}

void loop() {
  Serial.println("loop");
  //desni motor
  digitalWrite(MDP1, LOW);
  digitalWrite(MDP2, HIGH);
  ledcWrite(pwmChannelD, dutyCycleD);
  //levi motor
  digitalWrite(MLP3, LOW);
  digitalWrite(MLP4, HIGH);
  ledcWrite(pwmChannelL, dutyCycleL);
  Serial.println("vozim naprej");
  Serial.println("vozim naprej");
  delay(500);
}